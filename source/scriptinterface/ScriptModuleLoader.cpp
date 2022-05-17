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

#include "precompiled.h"

#include "ScriptModuleLoader.h"

#include "ScriptContext.h"
#include "ScriptConversions.h"
#include "ScriptExtraHeaders.h"
#include "ScriptInterface.h"

#include "lib/debug.h"
#include "lib/utf8.h"
#include "ps/CLogger.h"
#include "ps/CStr.h"
#include "ps/Filesystem.h"

JSObject* ScriptModuleLoader::CompileModule(JSContext* cx, const VfsPath& filePath) const
{
	ScriptRequest rq(cx);
	if (!VfsFileExists(filePath))
	{
		LOGERROR("File '%s' does not exist", filePath.string8());
		return nullptr;
	}

	if (filePath.Extension() != L".mjs" && false)
	{
		LOGERROR("File '%s' is not an ES module", filePath.string8());
		return nullptr;
	}

	CVFSFile file;

	PSRETURN ret = file.Load(g_VFS, filePath);
	if (ret != PSRETURN_OK)
	{
		LOGERROR("Failed to load file '%s': %s", filePath.string8(), GetErrorString(ret));
		return nullptr;
	}

	CStr code = file.DecodeUTF8();

	JS::CompileOptions options(cx);
	std::string filePathStr = filePath.string8();
	options.setFileAndLine(filePathStr.c_str(), 1);

	JS::SourceText<mozilla::Utf8Unit> src;
	ENSURE(src.init(cx, code.c_str(), code.length(), JS::SourceOwnership::Borrowed));

	JS::RootedObject mod(cx, JS::CompileModule(cx, options, src));
	JS::RootedObject modInfo(cx, JS_NewPlainObject(cx));
	JS::RootedValue modPath(cx);
	Script::ToJSVal(rq, &modPath, filePath.Parent());
	JS_DefineProperty(cx, modInfo, "path", modPath, JSPROP_ENUMERATE);
  	JS::SetModulePrivate(mod, JS::ObjectValue(*modInfo));

	return mod;
}

VfsPath ScriptModuleLoader::GetScriptPath(JSContext* cx, JS::HandleValue moduleHandle) const
{
	std::wstring ret;

	if (!moduleHandle.isUndefined())
	{
		ScriptRequest rq(cx);
		Script::FromJSProperty(rq, moduleHandle, "path", ret);
	}

	return VfsPath(ret);
}

JSObject* ScriptModuleLoader::ModuleResolutionHook(JSContext* cx, JS::HandleValue referencingModule, JS::HandleObject path)
{
	ScriptRequest rq(cx);
	const ScriptModuleLoader& instance = rq.GetScriptInterface().GetScriptModuleLoader();

	std::wstring pathString;
	JS::RootedValue pathValue(cx, JS::StringValue(JS::GetModuleRequestSpecifier(rq.cx, path)));
	Script::FromJSVal(rq, pathValue, pathString);

	VfsPath filename = VfsPath(pathString);
	VfsPath refFilename = instance.GetScriptPath(cx, referencingModule);

	VfsPath actualPath = (filename).Normalize();

	if (actualPath.Extension().empty())
		actualPath = actualPath.ChangeExtension(".js");

	std::map<VfsPath, JS::PersistentRootedObject>::iterator it = instance.m_ResolvedModules.find(actualPath);
	if (it != instance.m_ResolvedModules.end())
		return it->second;

	JS::RootedObject mod(cx, instance.CompileModule(cx, actualPath));

	if (!mod)
	{
		ScriptException::CatchPending(rq);
		return nullptr;
	}

	instance.m_ResolvedModules.emplace(actualPath, JS::PersistentRootedObject(cx, mod));

	return mod;
}

Status ScriptModuleLoader::FileChangedHook(void* param, const VfsPath& path)
{
	return static_cast<ScriptModuleLoader*>(param)->HandleChangedFile(path);
}

Status ScriptModuleLoader::HandleChangedFile(const VfsPath& path) const
{
	if (!VfsFileExists(path))
	{
		LOGERROR("File '%s' does not exist", path.string8());
		return ERR::FAIL;
	}

	if (path.Extension() != L".mjs" && false)
	{
		LOGERROR("File '%s' is not an ES module", path.string8());
		return ERR::FAIL;
	}

	std::map<VfsPath, JS::PersistentRootedObject>::iterator it = m_ResolvedModules.find(path);
	if (it != m_ResolvedModules.end())
		m_ResolvedModules.erase(it); // Let the next resolution recompile the module

	return INFO::OK;
}

bool MetadataHook(JSContext*, JS::HandleValue, JS::HandleObject UNUSED(metaObject))
{
	return true;
}

bool ScriptModuleLoader::Load(JSContext* cx, const VfsPath& modulePath)
{
	ScriptRequest rq(cx);

	JS::RootedObject global(cx, rq.glob);
	if (!global)
		return false;

	JS::SetModuleResolveHook(JS_GetRuntime(cx), ModuleResolutionHook);
	JS::SetModuleMetadataHook(JS_GetRuntime(cx), MetadataHook);

	JS::RootedObject topModule(cx, CompileModule(cx, modulePath));
	JS::RootedValue rval(cx);
	if (!topModule || !JS::ModuleInstantiate(cx, topModule) || !JS::ModuleEvaluate(cx, topModule, &rval))
	{
		ScriptException::CatchPending(rq);
		return false;
	}

	JS::RootedObject obj(rq.cx, rval.toObjectOrNull());
	if (obj)
		JS::ThrowOnModuleEvaluationFailure(cx, obj);

	ScriptException::CatchPending(rq);

	return true;
}
