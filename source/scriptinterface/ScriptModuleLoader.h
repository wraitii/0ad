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

#ifndef INCLUDED_SCRIPTMODULELOADER
#define INCLUDED_SCRIPTMODULELOADER

#include "lib/file/vfs/vfs_path.h"
#include "ps/Errors.h"
#include "scriptinterface/ScriptConversions.h"
#include "scriptinterface/ScriptExceptions.h"
#include "scriptinterface/ScriptRequest.h"
#include "scriptinterface/ScriptTypes.h"

#include <string>
#include <map>

class ScriptModuleLoader
{
public:
	ScriptModuleLoader() = default;
	~ScriptModuleLoader() = default;

	bool Load(JSContext* cx, const VfsPath& modulePath);

private:

	static JSObject* ModuleResolutionHook(JSContext* cx, JS::HandleValue referencingModule, JS::HandleString path);

	static Status FileChangedHook(void* param, const VfsPath& path);

	JSObject* CompileModule(JSContext* cx, const VfsPath& filePath) const;

	VfsPath GetScriptPath(JSContext* cx, JS::HandleValue moduleHandle) const;

	Status HandleChangedFile(const VfsPath& path) const;

	mutable std::map<VfsPath, JS::PersistentRootedObject> m_ResolvedModules;
};

#endif // INCLUDED_SCRIPTMODULELOADER
