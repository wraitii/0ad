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

#include "lib/self_test.h"

#include "ps/Future.h"
#include "ps/ThreadPool.h"

#include <atomic>
#include <condition_variable>
#include <mutex>

class TestThreadPool : public CxxTest::TestSuite
{
public:
	void test_basic()
	{
		ThreadPool::TaskManager taskManager;
		// There is a minimum of 3.
		TS_ASSERT_EQUALS(taskManager.GetNbOfWorkers(), 3);

		std::atomic<int> tasks_run = 0;
		auto increment_run = [&tasks_run]() { tasks_run++; };
		Future future = taskManager.GetWorker().Submit(increment_run);
		future.Wait();
		TS_ASSERT_EQUALS(tasks_run.load(), 1);

		// Test Execute.
		std::condition_variable cv;
		std::mutex mutex;
		std::atomic<bool> go = false;
		future = taskManager.GetWorker().Submit([&]() {
			std::unique_lock<std::mutex> lock(mutex);
			cv.wait(lock, [&go]() -> bool { return go; });
			lock.unlock();
			increment_run();
			go = false;
			cv.notify_all();
		});
		TS_ASSERT_EQUALS(tasks_run.load(), 1);
		go = true;
		cv.notify_all();
		std::unique_lock<std::mutex> lock(mutex);
		cv.wait(lock, [&go]() -> bool { return !go; });
		TS_ASSERT_EQUALS(tasks_run.load(), 2);
	}

	void test_PoolExecutor()
	{
		ThreadPool::TaskManager taskManager;
		// Push a blocked task on the worker so we can actually test the global queue.
		std::condition_variable cv;
		std::mutex mutex;
		bool go = false;
		taskManager.GetAllWorkers().begin()->Execute([&cv, &mutex, &go]() {
			std::unique_lock<std::mutex> lock(mutex);
			cv.wait(lock, [&go]() { return go; });
		});
		// Then push a general task.
		std::atomic<int> tasks_run = 0;
		auto increment_run = [&tasks_run]() { tasks_run++; };
		Future future = taskManager.GetExecutor().Submit(increment_run);
		go = true;
		cv.notify_all();
		future.Wait();
		TS_ASSERT_EQUALS(tasks_run.load(), 1);
		// Also check with no waiting expected.
		taskManager.GetExecutor().Submit(increment_run).Wait();
		TS_ASSERT_EQUALS(tasks_run.load(), 2);
	}

	// This isn't a perfect test, given the timer constraints, but it does.
	void test_TimerTasks()
	{
		ThreadPool::TaskManager taskManager;

		std::condition_variable cv;
		std::mutex mutex;

		std::atomic<int> counter = 0;
		taskManager.AddRecurrentTask(10, [&cv, &mutex, &counter](ThreadPool::GlobalExecutor<ThreadPool::Priority::NORMAL>&) mutable -> ThreadPool::RecurrentTaskStatus {
			{
				std::unique_lock<std::mutex> lock(mutex);
				counter++;
			}
			cv.notify_all();
			return counter == 10 ? ThreadPool::RecurrentTaskStatus::STOP : ThreadPool::RecurrentTaskStatus::OK;
		});
		{
			std::unique_lock<std::mutex> lock(mutex);
			cv.wait(lock, [&]() { return counter == 10; });
		}
	}
};
