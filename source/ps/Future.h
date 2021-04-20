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

#ifndef INCLUDED_FUTURE
#define INCLUDED_FUTURE

#include "ps/FutureForward.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <type_traits>

class IExecutor;

/**
 * Corresponds somewhat to std::packaged_task.
 * Like packaged_task, this holds a function acting as a promise.
 *
 * PackagedTask is blocking if the wrapped function has started processing
 * when it is destroyed, non-blocking otherwise.
 * This is intended as a safety net - the functions being wrapped are likely
 * to depend on some global state (profiler, logger, etc.) which may crash
 * if it ends up 'detached'. Since to run the packaged task, one must hold a reference
 * to it somewhere, we can use it to RAII this concern away.
 */
template<typename Ret>
class PackagedTask
{
	friend class Future<Ret>;
	static constexpr bool VoidReturn = std::is_same_v<Ret, void>;

	enum class Status
	{
		PENDING,
		STARTED,
		DONE,
		INVALID
	};

	class SharedState
	{
	public:
		SharedState(std::function<Ret()>&& func) : m_Func(func) {}
		~SharedState()
		{
			// For safety, wait on started task completion, but not on pending ones (auto-cancelled).
			if (!Cancel())
				Wait();
		}

		bool IsReady() const
		{
			return m_Status == Status::DONE || m_Status == Status::INVALID;
		}

		void Wait()
		{
			// Fast path: we're already done.
			if (IsReady())
				return;
			// Slow path: we aren't done when we run the above check. Lock and wait until we are.
			std::unique_lock<std::mutex> lock(m_Mutex);
			m_ConditionVariable.wait(lock, [this]() -> bool { return m_Status == Status::DONE; });
		}

		/**
		 * If the task is pending, cancel it by marking it as done.
		 * @return true if the task was indeed cancelled, false otherwise (the task is running or already done).
		 */
		bool Cancel()
		{
			if (IsReady())
				return false;
			Status expected = Status::PENDING;
			// If we're pending, try atomically setting to done.
			return m_Status.compare_exchange_strong(expected, Status::INVALID);
		}

		// Implemented below, accesses into IExecutor.
		void Continue();

		std::unique_ptr<IExecutor> m_Executor;
		std::atomic<Status> m_Status = Status::PENDING;
		std::mutex m_Mutex;
		std::condition_variable m_ConditionVariable;

		std::function<Ret()> m_Func;
		std::unique_ptr<std::function<void()>> m_Continuation;

		struct Empty {};
		union Result
		{
			std::aligned_storage_t<sizeof(Ret), alignof(Ret)> m_Bytes;
			Ret m_Result;
			Result() : m_Bytes() {};
		};
		// TODO C++20: [[no_unique_address]] this for a minor memory optimisation.
		// We don't use Result directly so the result doesn't have to be default constructible.
		std::conditional_t<VoidReturn, Empty, Result> m_Result;
	};

public:
	PackagedTask(std::shared_ptr<SharedState> ss) : m_SharedState(ss)
	{}

	PackagedTask(const PackagedTask&) = default;
	PackagedTask(PackagedTask&&) = default;

	~PackagedTask() {}

	void operator()()
	{
		Status expected = Status::PENDING;
		if (m_SharedState->m_Status.compare_exchange_strong(expected, Status::STARTED))
		{
			if constexpr (VoidReturn)
				m_SharedState->m_Func();
			else
				// To avoid UB, explicitly placement-new the value.
				new (&m_SharedState->m_Result) Ret{m_SharedState->m_Func()};

			if (m_SharedState->m_Continuation)
			{
				m_SharedState->Continue();
				return;
			}
			// Because we might have threads waiting on us, we need to make sure that they either:
			// - don't wait on our condition variable
			// - receive the notification when we're done.
			// This requires locking the mutex (@see Wait).
			{
				std::unique_lock<std::mutex> lock(m_SharedState->m_Mutex);
				m_SharedState->m_Status = Status::DONE;
			}

			m_SharedState->m_ConditionVariable.notify_all();
		}
	}

protected:
	std::shared_ptr<SharedState> m_SharedState;
};

/**
 * Corresponds to std::future (or more aptly the P1054r0 "ContinuableFuture" concept).
 * Unlike std::future, Future can request the cancellation of the task that would produce the result.
 * This makes it more similar to Java's CancellableTask or C#'s Task.
 * The name Future was kept over Task so it would be more familiar to C++ users,
 * but this all should be revised once Concurrency TS wraps up.
 *
 * The destructor is never blocking. The promise may still be running on destruction.
 * TODO:
 *  - Handle exceptions.
 */
template<typename Ret>
class Future
{
	friend class PackagedTask<Ret>;
	static constexpr bool VoidReturn = std::is_same_v<Ret, void>;
public:
	Future() = default;
	/**
	 * Only copyable before wrapping (and a no-op).
	 */
	Future(const Future&)
	{
		ENSURE(!Valid());
	}
	Future(Future&& o) = default;
	~Future() = default;

	Future& operator=(Future&& o) = default;

	/**
	 * Make the future wait for the result of @a func.
	 */
	template<typename T>
	PackagedTask<Ret> Wrap(std::unique_ptr<IExecutor>&& exec, T&& func)
	{
		static_assert(std::is_convertible_v<std::invoke_result_t<T>, Ret>, "The return type of the wrapped function cannot be converted to the type of the Future.");
		m_SharedState = std::make_shared<typename PackagedTask<Ret>::SharedState>(func);
		m_SharedState->m_Executor = std::move(exec);
		return PackagedTask<Ret>(m_SharedState);
	}

	/**
	 * Create a futureless executor - meant for testing, cannot be continued.
	 */
	template<typename T>
	PackagedTask<Ret> Wrap(T&& func)
	{
		static_assert(std::is_convertible_v<std::invoke_result_t<T>, Ret>, "The return type of the wrapped function cannot be converted to the type of the Future.");
		m_SharedState = std::make_shared<typename PackagedTask<Ret>::SharedState>(func);
		return PackagedTask<Ret>(m_SharedState);
	}

	template<typename T, typename... Args>
	Future<T> Then(std::function<T(Args...)>&& func) &&
	{
		ENSURE(m_SharedState->m_Executor);
		Future<T> fut;
		PackagedTask<T> task = fut.Wrap(std::move(m_SharedState->m_Executor), [func=func, input=m_SharedState]() mutable -> T {
			return func(std::move(input->m_Result.m_Result));
		});
		{
			std::unique_lock<std::mutex> lock(m_SharedState->m_Mutex);
			m_SharedState->m_Continuation = std::make_unique<std::function<void()>>(std::move(task));
		}
		if (m_SharedState->m_Status == PackagedTask<T>::Status::DONE)
			m_SharedState->Continue();
			return fut;
	}

	class BadFutureAccess : public std::exception
	{
		virtual const char* what() const noexcept
		{
			return "Tried to access the result of a future that was never wrapped or a cancelled future.";
		}
	};

	Ret Get()
	{
		Wait();
		if constexpr (VoidReturn)
			return;
		else
		{
			if (m_SharedState->m_Status == PackagedTask<Ret>::Status::INVALID)
			{
				// We were never wrapped/cancelled, throw.
				throw BadFutureAccess();
			}
			return m_SharedState->m_Result.m_Result;
		}
	}

	bool IsReady() const
	{
		return !!m_SharedState && m_SharedState->IsReady();
	}

	bool Valid() const
	{
		return !!m_SharedState;
	}

	void Wait()
	{
		if (Valid())
			m_SharedState->Wait();
	}

	void CancelOrWait()
	{
		if (!Valid())
			return;
		if (!Cancel())
			Wait();
	}

	/**
	 * Cancels the task. The result is always invalid, even if the task had completed before.
	 * Note that this cannot stop started tasks.
	 * @return True if the task was cancelled, false otherwise.
	 */
	bool Cancel()
	{
		if (!Valid())
			return false;
		return m_SharedState->Cancel();
	}
protected:
	std::shared_ptr<typename PackagedTask<Ret>::SharedState> m_SharedState;
};

/**
 * Executor interface. An executor is "something that schedules work".
 * TODO C++23: probably replaceable with standard versions when that lands in C++23 or later.
 */
class IExecutor
{
public:
	virtual ~IExecutor() = default;
	/**
	 * Submit a task in a fire-and-forget manner. The task is not guaranteed to run before program exit.
	 */
	void Execute(std::function<void()>&& func)
	{
		ExecuteTask(std::move(func));
	}
	/**
	 * Submit a task and get a future that will hold its result.
	 * The task is not guaranteed to run before program exit.
	 * Note that if you discard the future, the task is auto-cancelled,
	 * so use Execute for fire-and-forget tasks.
	 */
	template<typename T>
	[[nodiscard]] Future<std::invoke_result_t<T>> Submit(T&& func)
	{
		Future<std::invoke_result_t<T>> ret;
		ExecuteTask(ret.Wrap(Clone(), std::move(func)));
		return ret;
	}

protected:
	// Do the actual work.
	virtual void ExecuteTask(std::function<void()>&& task) = 0;
	virtual std::unique_ptr<IExecutor> Clone() const = 0;
};

template<typename Ret>
void PackagedTask<Ret>::SharedState::Continue()
{
	{
		std::unique_lock<std::mutex> lock(m_Mutex);
		m_Status = PackagedTask::Status::INVALID;
		m_Executor->Execute(std::move(*m_Continuation));
	}
	// Any thread that was waiting on the result of this shared state is likely in an invalid state -> wake them up to fail.
	m_ConditionVariable.notify_all();
}

#endif // INCLUDED_FUTURE
