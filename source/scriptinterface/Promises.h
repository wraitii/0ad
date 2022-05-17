/* Copyright (C) 2022 Wildfire Games.
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

#ifndef INCLUDED_SCRIPTINTERFACE_JOBQUEUE
#define INCLUDED_SCRIPTINTERFACE_JOBQUEUE

#include "js/Promise.h"

#include "scriptinterface/FunctionWrapper.h"
#include "scriptinterface/ScriptContext.h"
#include "scriptinterface/ScriptInterface.h"
#include "scriptinterface/ScriptRequest.h"
#include "scriptinterface/StructuredClone.h"

#include "ps/Filesystem.h"
#include "ps/TaskManager.h"

class ScriptInterface;

namespace Script
{
/**
 * Spidermonkey has to handle debugger interruptions to the job queue, which is a rather complex topic (see header).
 * We aren't going to care about that, only queuing jobs & running them.
 */
class JobQueue : public JS::JobQueue {
public:
	JobQueue() {};

	virtual ~JobQueue() = default;

	JSObject* getIncumbentGlobal(JSContext* cx)
	{
		return JS::CurrentGlobalOrNull(cx);
	}

	bool enqueuePromiseJob(JSContext* cx, JS::HandleObject UNUSED(promise), JS::HandleObject job, JS::HandleObject UNUSED(allocationSite), JS::HandleObject UNUSED(incumbentGlobal))
	{
		ScriptRequest rq(cx);
		const_cast<ScriptInterface&>(rq.GetScriptInterface()).AddJob(job);
		return true;
	}

	virtual void runJobs(JSContext*) {};

	virtual bool empty() const { return true; };

protected:
	// This is used by the debugger-interruptible queue.
	virtual js::UniquePtr<JS::JobQueue::SavedJobQueue> saveJobQueue(JSContext*) { return nullptr; };

private:
};

/**
 * Run a job in the thread pool. This returns a promise right away, and once the future completes will resolve the promise.
 */
template <auto callable, typename ...Args>
JS::HandleValue RunAsPromise(const ScriptInterface& scriptInterface, Args... args)
{
	ScriptRequest rq(scriptInterface);
	JS::RootedObject prom(rq.cx, JS::NewPromiseObject(rq.cx, nullptr));

	// Return a function to type-erase the return type of the callable.
	Future test = Threading::TaskManager::Instance().PushTask([args...]() -> std::function<JS::HandleValue(const ScriptRequest&)>{
		auto ret = callable(args...);
		return [ret] (const ScriptRequest& rq2) -> JS::HandleValue {
			JS::RootedValue val(rq2.cx);
			Script::ToJSVal(rq2, &val, ret);
			return val;
		};
	});
	JS::Heap<JSObject*> promObject(prom.get());
	const_cast<ScriptInterface&>(scriptInterface).AddPromise(promObject, std::move(test));

	JS::RootedValue promVal(rq.cx, JS::ObjectValue(*prom));
	// For some reason, this is needed of the returned object isn't interpreted as a promise
	// despite there apparently being no pending exception anyways?
	ScriptException::CatchPending(rq);

	return promVal;
}

/**
 * Run a job in the thread pool. This returns a promise right away, and once the future completes will resolve the promise.
 * This takes a VfsPath to a javascript file & some input data, and runs the job in a separate JS Context.
 * TODO: this is obviously massively slow compared to an efficient implementation.
 */
inline JS::HandleValue RunThreaded(const ScriptInterface& scriptInterface, std::wstring workerFile, JS::HandleValue callable)
{
	ScriptRequest rq(scriptInterface);
	JS::RootedObject prom(rq.cx, JS::NewPromiseObject(rq.cx, nullptr));

	Script::StructuredClone clone = Script::WriteStructuredClone(rq, callable);
	// Return a function to type-erase the return type of the callable.
	Future test = Threading::TaskManager::Instance().PushTask([workerFile, clone]() -> std::function<JS::HandleValue(const ScriptRequest&)>{
		// TODO: this is absurdly inefficient & only used for demonstration purposes.
		std::shared_ptr<ScriptContext> context = ScriptContext::CreateContext();
		ScriptInterface workerInterface("Engine", "worker context", context);
		ScriptRequest rqWorker(workerInterface);
		JS::RootedValue val(rqWorker.cx);
		Script::ReadStructuredClone(rqWorker, clone, &val);
		workerInterface.LoadGlobalScriptFile(VfsPath(workerFile));

		JS::RootedValue global(rqWorker.cx, rqWorker.globalValue());
		JS::RootedValue runResult(rqWorker.cx);
		ScriptFunction::Call(rqWorker, global, "run", &runResult, val);
		Script::StructuredClone resultClone = Script::WriteStructuredClone(rqWorker, runResult);

		return [resultClone] (const ScriptRequest& rq2) -> JS::HandleValue {
			JS::RootedValue resultVal(rq2.cx);
			Script::ReadStructuredClone(rq2, resultClone, &resultVal);
			return resultVal;
		};
	});
	JS::Heap<JSObject*> promObject(prom.get());
	const_cast<ScriptInterface&>(scriptInterface).AddPromise(promObject, std::move(test));

	JS::RootedValue promVal(rq.cx, JS::ObjectValue(*prom));
	// For some reason, this is needed of the returned object isn't interpreted as a promise
	// despite there apparently being no pending exception anyways?
	ScriptException::CatchPending(rq);

	return promVal;
}

}

#endif // INCLUDED_SCRIPTINTERFACE_JOBQUEUE
