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

#ifndef INCLUDED_THREADPOOL
#define INCLUDED_THREADPOOL

#include "ps/Future.h"

#include <memory>
#include <vector>

class TestThreadPool;
class CConfigDB;

namespace ThreadPool
{
class TaskManager;
class WorkerThread;

enum class Priority
{
	NORMAL,
	LOW
};

/**
 * Execute work on the global queue.
 * The low-priority queue has a differently-typed executor to highlight the different
 * starvation guaranteeds.
 */
template<Priority Priority = Priority::NORMAL>
class GlobalExecutor final : public IExecutor
{
	friend class TaskManager;
protected:
	GlobalExecutor(TaskManager& tm) : m_TaskManager(tm) {}

	virtual void ExecuteTask(std::function<void()>&& task) override;
	virtual std::unique_ptr<IExecutor> Clone() const override { return std::make_unique<GlobalExecutor>(*this); };

	TaskManager& m_TaskManager;
};

/**
 * Execute work directly a on thread's queue.
 */
class ThreadExecutor final : public IExecutor
{
	friend class EachThreadExecutor;
	friend class TaskManager;
protected:
	ThreadExecutor(WorkerThread& worker) : m_Worker(worker) {}

	virtual void ExecuteTask(std::function<void()>&& task) override;
	virtual std::unique_ptr<IExecutor> Clone() const override  { return std::make_unique<ThreadExecutor>(*this); };

	WorkerThread& m_Worker;
};

/**
 * Provides a convenient interface to iterating all worker threads.
 */
class EachThreadExecutor
{
	friend class TaskManager;
public:
	std::vector<ThreadExecutor>::iterator begin() { return m_Executors.begin(); }
	std::vector<ThreadExecutor>::iterator end() { return m_Executors.end(); }
private:
	std::vector<ThreadExecutor> m_Executors;
};

/**
 * TaskManager.
 */
class TaskManager
{
	friend class WorkerThread;
	template<Priority Priority>
	friend class GlobalExecutor;
public:
	TaskManager(size_t nbWorkers = 0);
	~TaskManager();
	TaskManager(const TaskManager&) = delete;
	TaskManager(TaskManager&&) = delete;

	static void Initialise();
	static TaskManager& Instance();

	/**
	 * Clears all tasks. This blocks on started tasks.
	 */
	void Clear();

	size_t GetNbOfWorkers() const;

	/**
	 * The global executor assigns work to the global queue, not a specific worker.
	 */
	GlobalExecutor<Priority::NORMAL> GetExecutor();
	GlobalExecutor<Priority::LOW> GetLowPriorityExecutor();

	/**
	 * Assigns work to a specific worker.
	 */
	ThreadExecutor GetWorker();

	/**
	 * Returns an executor that can be used to start (optionally different) work on (optionally all) threads.
	 */
	EachThreadExecutor& GetAllWorkers();

private:
	class Impl;
	std::unique_ptr<Impl> m;
};
} // ThreadPool

#endif // INCLUDED_THREADPOOL
