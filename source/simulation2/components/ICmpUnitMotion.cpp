/* Copyright (C) 2015 Wildfire Games.
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

#include "ICmpUnitMotion.h"

#include "simulation2/system/InterfaceScripted.h"
#include "simulation2/scripting/ScriptComponent.h"

BEGIN_INTERFACE_WRAPPER(UnitMotion)
DEFINE_INTERFACE_METHOD_4("SetNewDestinationAsPosition", bool, ICmpUnitMotion, SetNewDestinationAsPosition, entity_pos_t, entity_pos_t, entity_pos_t, bool)
DEFINE_INTERFACE_METHOD_3("SetNewDestinationAsEntity", bool, ICmpUnitMotion, SetNewDestinationAsEntity, entity_id_t, entity_pos_t, bool)
DEFINE_INTERFACE_METHOD_2("FaceTowardsPoint", void, ICmpUnitMotion, FaceTowardsPoint, entity_pos_t, entity_pos_t)
DEFINE_INTERFACE_METHOD_1("FaceTowardsEntity", void, ICmpUnitMotion, FaceTowardsEntity, entity_id_t)
DEFINE_INTERFACE_METHOD_1("SetAbortIfStuck", void, ICmpUnitMotion, SetAbortIfStuck, u8)
DEFINE_INTERFACE_METHOD_0("StopMoving", void, ICmpUnitMotion, StopMoving)
DEFINE_INTERFACE_METHOD_0("DiscardMove", void, ICmpUnitMotion, DiscardMove)
DEFINE_INTERFACE_METHOD_0("HasValidPath", bool, ICmpUnitMotion, HasValidPath)
DEFINE_INTERFACE_METHOD_0("GetTopSpeedRatio", fixed, ICmpUnitMotion, GetTopSpeedRatio)
DEFINE_INTERFACE_METHOD_1("SetSpeed", void, ICmpUnitMotion, SetSpeed, fixed)
DEFINE_INTERFACE_METHOD_0("IsActuallyMoving", bool, ICmpUnitMotion, IsActuallyMoving)
DEFINE_INTERFACE_METHOD_0("IsTryingToMove", bool, ICmpUnitMotion, IsTryingToMove)
DEFINE_INTERFACE_METHOD_0("GetSpeed", fixed, ICmpUnitMotion, GetSpeed)
DEFINE_INTERFACE_METHOD_0("GetBaseSpeed", fixed, ICmpUnitMotion, GetBaseSpeed)
DEFINE_INTERFACE_METHOD_0("GetPassabilityClassName", std::string, ICmpUnitMotion, GetPassabilityClassName)
DEFINE_INTERFACE_METHOD_0("GetUnitClearance", entity_pos_t, ICmpUnitMotion, GetUnitClearance)
DEFINE_INTERFACE_METHOD_1("SetFacePointAfterMove", void, ICmpUnitMotion, SetFacePointAfterMove, bool)
DEFINE_INTERFACE_METHOD_1("SetDebugOverlay", void, ICmpUnitMotion, SetDebugOverlay, bool)
END_INTERFACE_WRAPPER(UnitMotion)


class CCmpUnitMotionScripted : public ICmpUnitMotion
{
public:
	DEFAULT_SCRIPT_WRAPPER(UnitMotionScripted)

	virtual bool SetNewDestinationAsPosition(entity_pos_t x, entity_pos_t z, entity_pos_t range, bool UNUSED(evenUnreachable))
	{
		return m_Script.Call<bool>("SetNewDestinationAsPosition", x, z, range, true);
	}

	virtual bool SetNewDestinationAsEntity(entity_id_t target, entity_pos_t range, bool UNUSED(evenUnreachable))
	{
		return m_Script.Call<bool>("SetNewDestinationAsEntity", target, range, true);
	}

	virtual void FaceTowardsPoint(entity_pos_t x, entity_pos_t z)
	{
		m_Script.CallVoid("FaceTowardsPoint", x, z);
	}

	virtual void FaceTowardsEntity(entity_id_t ent)
	{
		m_Script.CallVoid("FaceTowardsEntity", ent);
	}

	virtual void DiscardMove()
	{
		m_Script.CallVoid("DiscardMove");
	}

	virtual void StopMoving()
	{
		m_Script.CallVoid("CompleteMove");
	}

	virtual void SetAbortIfStuck(u8 shouldAbort)
	{
		m_Script.CallVoid("SetAbortIfStuck", shouldAbort);
	}

	virtual fixed GetActualSpeed()
	{
		return m_Script.Call<fixed>("GetActualSpeed");
	}

	virtual void SetSpeed(fixed speed)
	{
		m_Script.CallVoid("SetSpeed", speed);
	}

	virtual fixed GetTopSpeedRatio()
	{
		return m_Script.Call<fixed>("GetTopSpeedRatio");
	}

	virtual bool HasValidPath()
	{
		return m_Script.Call<bool>("HasValidPath");
	}

	virtual bool IsActuallyMoving()
	{
		return m_Script.Call<bool>("IsActuallyMoving");
	}

	virtual bool IsTryingToMove()
	{
		return m_Script.Call<bool>("IsTryingToMove");
	}

	virtual fixed GetSpeed()
	{
		return m_Script.Call<fixed>("GetSpeed");
	}

	virtual fixed GetBaseSpeed()
	{
		return m_Script.Call<fixed>("GetBaseSpeed");
	}

	virtual void SetFacePointAfterMove(bool facePointAfterMove)
	{
		m_Script.CallVoid("SetFacePointAfterMove", facePointAfterMove);
	}

	virtual pass_class_t GetPassabilityClass()
	{
		return m_Script.Call<pass_class_t>("GetPassabilityClass");
	}

	virtual fixed GetSpeedRatio()
	{
		return fixed::FromInt(1);
	}

	virtual std::string GetPassabilityClassName()
	{
		return m_Script.Call<std::string>("GetPassabilityClassName");
	}

	virtual entity_pos_t GetUnitClearance()
	{
		return m_Script.Call<entity_pos_t>("GetUnitClearance");
	}

	virtual void SetDebugOverlay(bool enabled)
	{
		m_Script.CallVoid("SetDebugOverlay", enabled);
	}

};

REGISTER_COMPONENT_SCRIPT_WRAPPER(UnitMotionScripted)
