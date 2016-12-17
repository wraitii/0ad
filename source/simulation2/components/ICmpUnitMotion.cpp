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
DEFINE_INTERFACE_METHOD_4("MoveToPointRange", bool, ICmpUnitMotion, MoveToPointRange, entity_pos_t, entity_pos_t, entity_pos_t, entity_pos_t)
DEFINE_INTERFACE_METHOD_4("IsInPointRange", bool, ICmpUnitMotion, IsInPointRange, entity_pos_t, entity_pos_t, entity_pos_t, entity_pos_t)
DEFINE_INTERFACE_METHOD_3("IsInTargetRange", bool, ICmpUnitMotion, IsInTargetRange, entity_id_t, entity_pos_t, entity_pos_t)
DEFINE_INTERFACE_METHOD_3("MoveToTargetRange", bool, ICmpUnitMotion, MoveToTargetRange, entity_id_t, entity_pos_t, entity_pos_t)
DEFINE_INTERFACE_METHOD_2("FaceTowardsPoint", void, ICmpUnitMotion, FaceTowardsPoint, entity_pos_t, entity_pos_t)
DEFINE_INTERFACE_METHOD_1("SetAbordIfStuck", void, ICmpUnitMotion, SetAbortIfStuck, bool)
DEFINE_INTERFACE_METHOD_0("DiscardMove", void, ICmpUnitMotion, DiscardMove)
DEFINE_INTERFACE_METHOD_0("CompleteMove", void, ICmpUnitMotion, CompleteMove)
DEFINE_INTERFACE_METHOD_0("GetActualSpeed", fixed, ICmpUnitMotion, GetActualSpeed)
DEFINE_INTERFACE_METHOD_1("SetSpeed", void, ICmpUnitMotion, SetSpeed, fixed)
DEFINE_INTERFACE_METHOD_0("IsActuallyMoving", bool, ICmpUnitMotion, IsActuallyMoving)
DEFINE_INTERFACE_METHOD_0("IsTryingToMove", bool, ICmpUnitMotion, IsTryingToMove)
DEFINE_INTERFACE_METHOD_0("GetSpeed", fixed, ICmpUnitMotion, GetSpeed)
DEFINE_INTERFACE_METHOD_0("GetTemplateSpeed", fixed, ICmpUnitMotion, GetTemplateSpeed)
DEFINE_INTERFACE_METHOD_0("GetPassabilityClassName", std::string, ICmpUnitMotion, GetPassabilityClassName)
DEFINE_INTERFACE_METHOD_0("GetUnitClearance", entity_pos_t, ICmpUnitMotion, GetUnitClearance)
DEFINE_INTERFACE_METHOD_1("SetFacePointAfterMove", void, ICmpUnitMotion, SetFacePointAfterMove, bool)
DEFINE_INTERFACE_METHOD_1("SetDebugOverlay", void, ICmpUnitMotion, SetDebugOverlay, bool)
END_INTERFACE_WRAPPER(UnitMotion)

class CCmpUnitMotionScripted : public ICmpUnitMotion
{
public:
	DEFAULT_SCRIPT_WRAPPER(UnitMotionScripted)

	virtual bool MoveToPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange)
	{
		return m_Script.Call<bool>("MoveToPointRange", x, z, minRange, maxRange);
	}

	virtual bool IsInPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange)
	{
		return m_Script.Call<bool>("IsInPointRange", x, z, minRange, maxRange);
	}

	virtual bool IsInTargetRange(entity_id_t target, entity_pos_t minRange, entity_pos_t maxRange)
	{
		return m_Script.Call<bool>("IsInTargetRange", target, minRange, maxRange);
	}

	virtual bool MoveToTargetRange(entity_id_t target, entity_pos_t minRange, entity_pos_t maxRange)
	{
		return m_Script.Call<bool>("MoveToTargetRange", target, minRange, maxRange);
	}

	virtual void FaceTowardsPoint(entity_pos_t x, entity_pos_t z)
	{
		m_Script.CallVoid("FaceTowardsPoint", x, z);
	}

	virtual void DiscardMove()
	{
		m_Script.CallVoid("DiscardMove");
	}

	virtual void CompleteMove()
	{
		m_Script.CallVoid("CompleteMove");
	}

	virtual void SetAbortIfStuck(bool shouldAbort)
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

	virtual fixed GetTemplateSpeed()
	{
		return m_Script.Call<fixed>("GetTemplateSpeed");
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
