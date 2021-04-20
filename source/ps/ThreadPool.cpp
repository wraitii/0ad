/* Copyright (C) 2021 Wildfire Games.
 * This file is part of 0 A.D.
 *
 * 0 A.D. is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * 0 A.D. is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 0 A.D.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "precompiled.h"

#include "ThreadPool.h"

#include "lib/debug.h"
#include "maths/MathUtil.h"
#include "ps/CLogger.h"
#include "ps/ConfigDB.h"
#include "ps/Threading.h"
#include "ps/ThreadUtil.h"
#include "ps/Profiler2.h"

#include <condition_variable>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <list>
#include <shared_mutex>
#include <thread>

namespace ThreadPool
{
/**
 * Minimum number of thread pool workers.
 */
static constexpr size_t MIN_THREADS = 3;

/**
 * Maximum number of thread pool workers.
 */
static constexpr size_t MAX_THREADS = 32;

/**
 * Returns the # of 'reversed priority' workers to use.
 * See comment on avoiding starvation in TaskManager.
 * At least one such worker will always exist.
 */
constexpr size_t GetNbOfReversedPriorityWorker(size_t nbWorkers)
{
	size_t ret = nbWorkers * 0.25;
	return ret > 1 ? ret : 1;
}

std::unique_ptr<TaskManager> g_TaskManager;

/**
 * Run the timer thread every X.
 * NB: this is currently allowed to drift over time, as that's presumed not too important.
 * This value should be high enough that tasks can execute somewhat real-time,
 * but low enough that the thread is largely idle & won't have negative scheduling effects.
 */
static constexpr std::chrono::microseconds TIMER_INTERVAL = std::chrono::microseconds(30000);

/**
 * The timer thread runs recurrent tasks on its own thread to avoid context switching.
 * This means these tasks must be fast.
 * When the wakup takes more than this much time, print a warning.
 * This is conservative, it's meant to detect code errors.
 */
static constexpr std::chrono::microseconds MAX_TIMER_WAKUP_TIME = std::chrono::microseconds(1000);

class Thread;
class TimerThread;
}

using QueueItem = std::function<void()>;

/**
 * Light wrapper around std::thread. Ensures Join has been called.
 */
class ThreadPool::Thread
{
public:
	Thread() = default;
	Thread(const Thread&) = delete;
	Thread(Thread&&) = delete;

	template<typename T, void(T::* callable)()>
	void Start(T* object)
	{
		m_Thread = std::thread(Threading::HandleExceptions<DoStart<T, callable>>::Wrapper, object);
	}
	template<typename T, void(T::* callable)()>
	static void DoStart(T* object);

protected:
	~Thread()
	{
		ENSURE(!m_Thread.joinable());
	}

	std::thread m_Thread;
	std::atomic<bool> m_Kill = false;
};

/**
 * Thread with a work queue that it processes until it's destroyed.
 */
class ThreadPool::WorkerThread : public Thread
{
public:
	WorkerThread(ThreadPool::TaskManager::Impl& taskManager, bool reversePriority = false);
	~WorkerThread();

	/**
	 * Clear the task queue.
	 */
	void Clear();

	/**
	 * Add a task to this worker's queue.
	 * Takes ownership of the task.
	 * May be called from any thread.
	 */
	void PushTask(QueueItem&& task);

protected:
	template<bool reversePriority>
	void RunUntilDeath();

	std::mutex m_OwnQueueMutex;
	std::deque<QueueItem> m_Queue;

	// Shared by all worker threads
	std::shared_mutex& m_Mutex;
	std::condition_variable_any& m_ConditionVariable;
	ThreadPool::TaskManager::Impl& m_TaskManager;
};

class ThreadPool::TimerThread final : public Thread
{
public:
	TimerThread(const GlobalExecutor<Priority::NORMAL>& exec) : m_Executor(exec)
	{
		Start<TimerThread, &TimerThread::RunUntilDeath>(this);
	};

	~TimerThread()
	{
		m_Kill = true;
		Clear();
		if (m_Thread.joinable())
			m_Thread.join();
	}

	/**
	 * Clear recurrent & one-off tasks.
	 */
	void Clear()
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		m_RecurrentTasks.clear();
	}

	/**
	 * Add a recurrent task. The task will be run on the next timer call.
	 * @param repeat - How often to run this task. 0 is every time, 1 is every 2, 2 every 3, and so on.
	 * @param func - Task to run.
	 */
	void PushRecurrentTask(u32 repeat, std::function<RecurrentTaskStatus(GlobalExecutor<Priority::NORMAL>&)>&& func)
	{
		m_RecurrentTasks.emplace_back(repeat, std::move(func));
	}

protected:
	void RunUntilDeath()
	{
		std::string name("ThreadPool Timer");
		g_Profiler2.RegisterCurrentThread(name);
		debug_SetThreadName(name.c_str());

		std::chrono::microseconds sleepTime = TIMER_INTERVAL;
		std::chrono::steady_clock clock;
		std::chrono::time_point t0 = clock.now();
		u32 warningCounter = 0;
		std::vector<size_t> indicesToDrop;

		std::unique_lock<std::mutex> lock(m_Mutex);
		while (!m_Kill)
		{
			// Tolerate spurious wake ups.
			m_ConditionVariable.wait_for(lock, sleepTime);
			if (m_Kill)
				break;
			// We give no guarantees on actual timing (because that's much easier),
			// so just fire tasks.
			t0 = clock.now();

			indicesToDrop.clear();
			for (size_t i = 0; i < m_RecurrentTasks.size(); ++i)
			{
				RecurrentTask& task = m_RecurrentTasks[i];
				if (task.m_Cooldown == 0)
				{
					RecurrentTaskStatus status = task.m_Func(m_Executor);
					if (status == RecurrentTaskStatus::OK)
						task.m_Cooldown = task.m_Repeat;
					else if (status == RecurrentTaskStatus::STOP)
						indicesToDrop.push_back(i);
					// Else keep as-is for retry.
				}
				else
					--task.m_Cooldown;
			}
			for (size_t idx : indicesToDrop)
			{
				m_RecurrentTasks[idx] = std::move(m_RecurrentTasks.back());
				m_RecurrentTasks.pop_back();
			}
			indicesToDrop.clear();

			auto wakeTime = clock.now() - t0;
			// Since it's measured, correct for run time, but this won't prevent
			// actual drift over time.
			sleepTime = std::chrono::duration_cast<std::chrono::microseconds>(TIMER_INTERVAL - wakeTime);

			// Warning logic - we want to warn if the timer is too often too slow
			// since that probably indicates a mistake somewhere (e.g. the wrong function is run).
			// However we don't want spurious warnings, so add 2 for each overrun, and remove 1 otherwise.
			// This will warn if enough runs are slow in a short time period.
			if (wakeTime > MAX_TIMER_WAKUP_TIME)
			{
				warningCounter += 2;
				if (warningCounter > 50)
				{
					warningCounter = 0;
					LOGWARNING("ThreadPool Timer: starting tasks took %ims, which is more than the limit of %ims",
					           std::chrono::duration_cast<std::chrono::milliseconds>(sleepTime).count(),
					           std::chrono::duration_cast<std::chrono::milliseconds>(MAX_TIMER_WAKUP_TIME).count());
				}
			}
			else if (warningCounter > 0)
				--warningCounter;
		}
	}

	std::mutex m_Mutex;
	std::condition_variable m_ConditionVariable;

	GlobalExecutor<Priority::NORMAL> m_Executor;

	struct RecurrentTask
	{
		RecurrentTask(u32 repeat, std::function<RecurrentTaskStatus(GlobalExecutor<Priority::NORMAL>&)>&& func) : m_Repeat(repeat), m_Func(std::move(func)) {}
		u32 m_Repeat = 0;
		u32 m_Cooldown = 0;
		std::function<RecurrentTaskStatus(GlobalExecutor<Priority::NORMAL>&)> m_Func;
	};
	std::vector<RecurrentTask> m_RecurrentTasks;
};

/**
 * PImpl-ed implementation of the threadpool's task manager.
 *
 * Avoiding starvation:
 * To keep things simple while avoiding starvation,
 * some workers will process the global queue before their own.
 * These workers won't be returned when asking for a single-worker executor.
 * The low-priority queue is not guaranteed to run.
 */
class ThreadPool::TaskManager::Impl
{
	friend class TaskManager;
	friend class WorkerThread;
public:
	Impl(TaskManager& backref, size_t nbWorkers);
	~Impl()
	{
		Clear();
		m_Workers.clear();
	}

	/**
	 * Push a task on the global queue.
	 * Takes ownership of @a task.
	 * May be called from any thread.
	 */
	template<Priority Priority>
	static void PushTask(TaskManager::Impl& taskManager, QueueItem&& task);

protected:
	void Clear();
	ThreadExecutor GetWorker();

	// Back reference (keep this first).
	TaskManager& m_TaskManager;

	// Single-producer, multiple-consumer queue -> use a shared_mutex.
	std::shared_mutex m_Mutex;
	std::condition_variable_any m_ConditionVariable;

	std::deque<QueueItem> m_GlobalQueue;
	std::deque<QueueItem> m_GlobalLowPriorityQueue;
	std::atomic<bool> m_HasWork;
	std::atomic<bool> m_HasLowPriorityWork;

	std::list<WorkerThread> m_Workers;
	TimerThread m_TimerWorker;

	// This does not contain all workers, see comment on avoiding starvation above.
	std::vector<ThreadExecutor> m_DedicatedExecutors;
	EachThreadExecutor m_EachThreadExecutor;

	// Round-robin counter for GetWorker.
	size_t m_RoundRobinIdx = 0;
	size_t m_FirstReversedIdx;
};

ThreadPool::TaskManager::TaskManager(size_t nbWorkers)
{
	if (nbWorkers < MIN_THREADS)
		nbWorkers = Clamp<size_t>(std::thread::hardware_concurrency() - 1, MIN_THREADS, MAX_THREADS);

	m = std::make_unique<Impl>(*this, nbWorkers);
}

ThreadPool::TaskManager::~TaskManager() {}

ThreadPool::TaskManager::Impl::Impl(TaskManager& backref, size_t nbWorkers)
	: m_TaskManager(backref), m_TimerWorker(m_TaskManager.GetExecutor())
{
	m_FirstReversedIdx = nbWorkers - GetNbOfReversedPriorityWorker(nbWorkers);
	for (size_t i = 0; i < nbWorkers; ++i)
	{
		bool reversePriority = i >= m_FirstReversedIdx;
		WorkerThread& worker = m_Workers.emplace_back(*this, reversePriority);
		if (!reversePriority)
			m_DedicatedExecutors.emplace_back(ThreadExecutor(worker));
		m_EachThreadExecutor.m_Executors.emplace_back(ThreadExecutor(worker));
		// Register the thread on Profiler2 and name it, for convenience.
		// (This is called here because RunUntilDeath is templated and the static value won't work as expected then).
		m_EachThreadExecutor.m_Executors.back().Submit([]() {
			// The profiler does better if the names are unique.
			static std::atomic<int> n = 0;
			std::string name = "ThreadPool #" + std::to_string(n++);
			debug_SetThreadName(name.c_str());
			g_Profiler2.RegisterCurrentThread(name);
		}).Wait();
	}
}

void ThreadPool::TaskManager::Clear() { m->Clear(); }
void ThreadPool::TaskManager::Impl::Clear()
{
	m_TimerWorker.Clear();
	for (WorkerThread& worker : m_Workers)
		worker.Clear();
}

size_t ThreadPool::TaskManager::GetNbOfWorkers() const
{
	return m->m_Workers.size();
}

ThreadPool::GlobalExecutor<ThreadPool::Priority::NORMAL> ThreadPool::TaskManager::GetExecutor()
{
	return *this;
}

ThreadPool::GlobalExecutor<ThreadPool::Priority::LOW> ThreadPool::TaskManager::GetLowPriorityExecutor()
{
	return *this;
}

ThreadPool::EachThreadExecutor& ThreadPool::TaskManager::GetAllWorkers()
{
	return m->m_EachThreadExecutor;
}

ThreadPool::ThreadExecutor ThreadPool::TaskManager::GetWorker() { return m->GetWorker(); }
ThreadPool::ThreadExecutor ThreadPool::TaskManager::Impl::GetWorker()
{
	if (m_RoundRobinIdx >= m_FirstReversedIdx)
		m_RoundRobinIdx = 0;
	return m_DedicatedExecutors[m_RoundRobinIdx];
}

template<ThreadPool::Priority Priority>
void ThreadPool::TaskManager::Impl::PushTask(TaskManager::Impl& taskManager, QueueItem&& task)
{
	std::lock_guard<std::shared_mutex> lock(taskManager.m_Mutex);
	if constexpr (Priority == Priority::NORMAL)
	{
		taskManager.m_HasWork = true;
		taskManager.m_GlobalQueue.push_back(std::move(task));
	}
	else
	{
		taskManager.m_HasLowPriorityWork = true;
		taskManager.m_GlobalLowPriorityQueue.push_back(std::move(task));
	}

	// Notify one thread to wake up and process this task.
	// Note that this works fine even with the 2 types of worker threads,
	// because it will wake a waiting thread, and waiting threads have nothing better to do.
	taskManager.m_ConditionVariable.notify_one();
}

void ThreadPool::TaskManager::Initialise()
{
	if (!g_TaskManager)
		g_TaskManager = std::make_unique<TaskManager>();
}

ThreadPool::TaskManager& ThreadPool::TaskManager::Instance()
{
	ENSURE(g_TaskManager);
	return *g_TaskManager;
}

// Thread definition

ThreadPool::WorkerThread::WorkerThread(ThreadPool::TaskManager::Impl& taskManager, bool reversePriority)
	: m_TaskManager(taskManager), m_Mutex(taskManager.m_Mutex), m_ConditionVariable(taskManager.m_ConditionVariable)
{
	// Explicitness is required or the compiler gets confused about types.
	if (reversePriority)
		Start<WorkerThread, &WorkerThread::RunUntilDeath<true>>(this);
	else
		Start<WorkerThread, &WorkerThread::RunUntilDeath<false>>(this);
}

ThreadPool::WorkerThread::~WorkerThread()
{
	Clear();
	m_Kill = true;
	m_ConditionVariable.notify_all();
	if (m_Thread.joinable())
		m_Thread.join();
}

void ThreadPool::WorkerThread::Clear()
{
	std::unique_lock<std::mutex> lock(m_OwnQueueMutex);
	m_Queue.clear();
}

void ThreadPool::WorkerThread::PushTask(QueueItem&& task)
{
	{
		std::unique_lock<std::mutex> lock(m_OwnQueueMutex);
		m_Queue.push_back(std::move(task));
	}
	m_ConditionVariable.notify_all();
}

template<bool reversePriority>
void ThreadPool::WorkerThread::RunUntilDeath()
{
	std::shared_lock<std::shared_mutex> lock(m_Mutex, std::defer_lock);
	std::unique_lock<std::mutex> ownLock(m_OwnQueueMutex, std::defer_lock);
	while (!m_Kill)
	{
		if (!lock.owns_lock())
			lock.lock();
		m_ConditionVariable.wait(lock, [this]() -> bool {
			std::unique_lock<std::mutex> lock(m_OwnQueueMutex);
			if (!m_Queue.empty() || m_Kill)
				return true;
			return m_TaskManager.m_HasWork || m_TaskManager.m_HasLowPriorityWork;
		});
		if constexpr (!reversePriority)
		{
			ownLock.lock();
			if (!m_Kill && !m_Queue.empty())
			{
				lock.unlock();
				QueueItem task = std::move(m_Queue.front());
				m_Queue.pop_front();
				ownLock.unlock();
				task();
				continue;
			}
			else
				ownLock.unlock();
		}
		if (!m_Kill && m_TaskManager.m_HasWork)
		{
			// Check the global pool queue.
			if (!m_TaskManager.m_GlobalQueue.empty())
			{
				lock.unlock();
				std::unique_lock<std::shared_mutex> writeLock(m_TaskManager.m_Mutex);
				QueueItem task = std::move(m_TaskManager.m_GlobalQueue.front());
				m_TaskManager.m_GlobalQueue.pop_front();
				if (m_TaskManager.m_GlobalQueue.empty())
					m_TaskManager.m_HasWork = false;
				writeLock.unlock();
				task();
				continue;
			}
		}
		// Duplicated above.
		if constexpr (reversePriority)
		{
			ownLock.lock();
			if (!m_Kill && !m_Queue.empty())
			{
				lock.unlock();
				QueueItem task = std::move(m_Queue.front());
				m_Queue.pop_front();
				ownLock.unlock();
				task();
				continue;
			}
			else
				ownLock.unlock();
		}
		if (!m_Kill && m_TaskManager.m_HasLowPriorityWork)
		{
			// Check the global pool queue.
			if (!m_TaskManager.m_GlobalLowPriorityQueue.empty())
			{
				lock.unlock();
				std::unique_lock<std::shared_mutex> writeLock(m_TaskManager.m_Mutex);
				QueueItem task = std::move(m_TaskManager.m_GlobalLowPriorityQueue.front());
				m_TaskManager.m_GlobalLowPriorityQueue.pop_front();
				if (m_TaskManager.m_GlobalLowPriorityQueue.empty())
					m_TaskManager.m_HasLowPriorityWork = false;
				writeLock.unlock();
				task();
				continue;
			}
		}
	}
}

// Executor implementation below

template<ThreadPool::Priority Priority>
void ThreadPool::GlobalExecutor<Priority>::ExecuteTask(QueueItem&& task)
{
	TaskManager::Impl::PushTask<Priority>(*m_TaskManager.m, std::move(task));
}

void ThreadPool::TaskManager::AddRecurrentTask(u32 repeatms, std::function<RecurrentTaskStatus(GlobalExecutor<Priority::NORMAL>&)>&& func)
{
	u32 repeat = round(std::chrono::duration<double, std::milli>(repeatms) / TIMER_INTERVAL);
	// Need to remove one additional tick as 0 means "every timer"
	if (repeat > 0)
		--repeat;
	m->m_TimerWorker.PushRecurrentTask(repeat, std::move(func));
}

void ThreadPool::ThreadExecutor::ExecuteTask(QueueItem&& task)
{
	m_Worker.PushTask(std::move(task));
}

template<typename T, void(T::* callable)()>
void ThreadPool::Thread::DoStart(T* object)
{
	std::apply(callable, std::forward_as_tuple(object));
}

