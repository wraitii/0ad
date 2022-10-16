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

/*
 * Container that owns all units
 */

#include "precompiled.h"

#include <float.h>

#include "Model.h"
#include "UnitManager.h"
#include "Unit.h"
#include "ObjectManager.h"
#include "ObjectEntry.h"

#include "maths/Frustum.h"
#include "maths/Matrix3D.h"
#include "maths/MathUtil.h"
#include "ps/Game.h"
#include "ps/World.h"
#include "renderer/RenderingOptions.h"
#include "tools/atlas/GameInterface/GameLoop.h"

#include <algorithm>

///////////////////////////////////////////////////////////////////////////////
// CUnitManager constructor
CUnitManager::CUnitManager() :
	m_ObjectManager(NULL)
{
	m_FrameNumber = 0;
	m_FrameOffset = 0.0f;
	m_EnableDebugOverlays = false;
}

///////////////////////////////////////////////////////////////////////////////
// CUnitManager destructor
CUnitManager::~CUnitManager()
{
	DeleteAll();
}


///////////////////////////////////////////////////////////////////////////////
// AddUnit: add given unit to world
void CUnitManager::AddUnit(CUnit* unit)
{
	m_Units.push_back(unit);
}

///////////////////////////////////////////////////////////////////////////////
// RemoveUnit: remove given unit from world, but don't delete it
void CUnitManager::RemoveUnit(CUnit* unit)
{
	// find entry in list
	typedef std::vector<CUnit*>::iterator Iter;
	Iter i=std::find(m_Units.begin(),m_Units.end(),unit);
	if (i!=m_Units.end()) {
		m_Units.erase(i);
	}
}

///////////////////////////////////////////////////////////////////////////////
// DeleteUnit: remove given unit from world and delete it
void CUnitManager::DeleteUnit(CUnit* unit)
{
	RemoveUnit(unit);
	delete unit;
}

///////////////////////////////////////////////////////////////////////////////
// DeleteAll: remove and delete all units
void CUnitManager::DeleteAll()
{
	for (size_t i=0;i<m_Units.size();i++) {
		delete m_Units[i];
	}
	m_Units.clear();
}

///////////////////////////////////////////////////////////////////////////////
// CreateUnit: create a new unit and add it to the world
CUnit* CUnitManager::CreateUnit(const CStrW& actorName, uint32_t seed)
{
	if (! m_ObjectManager)
		return NULL;

	CUnit* unit = CUnit::Create(actorName, seed, *m_ObjectManager);
	if (unit)
		AddUnit(unit);
	return unit;
}

void CUnitManager::MakeTerrainDirty(ssize_t i0, ssize_t j0, ssize_t i1, ssize_t j1, int dirtyFlags)
{
	if (!(dirtyFlags & RENDERDATA_UPDATE_VERTICES))
		return;
	for (CUnit* unit : m_Units)
		unit->GetModel().SetTerrainDirty(i0, j0, i1, j1);
}

CUnitManager::SUnit* CUnitManager::LookupUnit(tag_t tag)
{
	if (tag.n < 1 || tag.n - 1 >= m_UnitsData.size())
		return NULL;
	return &m_UnitsData[tag.n - 1];
}

CUnitManager::tag_t CUnitManager::AddUnit(CEntityHandle entity, CUnit* actor, const CBoundingSphere& boundsApprox, int flags)
{
	ENSURE(actor != NULL);

	tag_t tag;
	if (!m_UnitTagsFree.empty())
	{
		tag = m_UnitTagsFree.back();
		m_UnitTagsFree.pop_back();
	}
	else
	{
		m_UnitsData.push_back(SUnit());
		tag.n = m_UnitsData.size();
	}

	SUnit* unit = LookupUnit(tag);
	unit->entity_id = entity.GetId();
	unit->actor = actor;
	unit->lastTransformFrame = -1;
	unit->flags = flags;
	unit->boundsApprox = boundsApprox;
	unit->inWorld = false;
	unit->visibilityDirty = true;
	unit->pos0 = unit->pos1 = CVector3D();

	return tag;
}


void CUnitManager::RemoveUnit(tag_t tag)
{
	SUnit* unit = LookupUnit(tag);
	unit->actor = NULL;
	unit->inWorld = false;
	m_UnitTagsFree.push_back(tag);
}

void CUnitManager::RecomputeSweptBounds(SUnit* unit)
{
	// Compute the bounding sphere of the capsule formed by
	// sweeping boundsApprox from pos0 to pos1
	CVector3D mid = (unit->pos0 + unit->pos1) * 0.5f + unit->boundsApprox.GetCenter();
	float radius = (unit->pos1 - unit->pos0).Length() * 0.5f + unit->boundsApprox.GetRadius();
	unit->sweptBounds = CBoundingSphere(mid, radius);
}

void CUnitManager::UpdateUnit(tag_t tag, CUnit* actor, const CBoundingSphere& boundsApprox)
{
	SUnit* unit = LookupUnit(tag);
	unit->actor = actor;
	unit->boundsApprox = boundsApprox;
	RecomputeSweptBounds(unit);
}

void CUnitManager::UpdateUnitPos(tag_t tag, bool inWorld, const CVector3D& pos0, const CVector3D& pos1)
{
	SUnit* unit = LookupUnit(tag);
	unit->inWorld = inWorld;
	unit->pos0 = pos0;
	unit->pos1 = pos1;
	unit->visibilityDirty = true;
	RecomputeSweptBounds(unit);
}

float CUnitManager::GetFrameOffset() const
{
	return m_FrameOffset;
}

void CUnitManager::SetDebugOverlay(bool enabled)
{
	m_EnableDebugOverlays = enabled;
}

void CUnitManager::PickAllEntitiesAtPoint(std::vector<std::pair<CEntityHandle, CVector3D> >& outEntities, const CVector3D& origin, const CVector3D& dir, bool allowEditorSelectables) const
{
	/*
	// First, make a rough test with the worst-case bounding boxes to pick all
	// entities/models that could possibly be hit by the ray.
	std::vector<const SUnit*> candidates;
	for (const SUnit& unit : m_UnitsData)
	{
		if (!unit.actor || !unit.inWorld)
			continue;
		if (unit.sweptBounds.RayIntersect(origin, dir))
			candidates.push_back(&unit);
	}

	// Now make a more precise test to get rid of the remaining false positives
	float tmin, tmax;
	CVector3D center;
	for (size_t i = 0; i< candidates.size(); ++i)
	{
		const SUnit& unit = *candidates[i];

		CmpPtr<ICmpVisual> cmpVisual(unit.entity);
		if (!cmpVisual)
			continue;

		CBoundingBoxOriented selectionBox = cmpVisual->GetSelectionBox();
		if (selectionBox.IsEmpty())
		{
			if (!allowEditorSelectables)
				continue;

			// Fall back to using old AABB selection method for decals
			//	see: http://trac.wildfiregames.com/ticket/1032
			// Decals are flat objects without a selectionShape defined,
			// but they should still be selectable in the editor to move them
			// around or delete them after they are placed.
			// Check campaigns/labels/ in the Actors tab of atlas for examples.
			CBoundingBoxAligned aABBox = cmpVisual->GetBounds();
			if (aABBox.IsEmpty())
				continue;

			if (!aABBox.RayIntersect(origin, dir, tmin, tmax))
				continue;

			aABBox.GetCenter(center);
		}
		else
		{
			if (!selectionBox.RayIntersect(origin, dir, tmin, tmax))
				continue;

			center = selectionBox.m_Center;
		}
		outEntities.emplace_back(unit.entity, center);
	}
	 */
}

void CUnitManager::TurnStart()
{
	PROFILE3("CUnitManager::TurnStart");

	// Assume units have stopped moving after the previous turn. If that assumption is not
	// correct, we will get a UpdateUnitPos to tell us about its movement in the new turn.
	for (size_t i = 0; i < m_UnitsData.size(); i++)
	{
		SUnit& unit = m_UnitsData[i];
		unit.pos0 = unit.pos1;
		unit.sweptBounds = CBoundingSphere(unit.pos1, unit.boundsApprox.GetRadius());

		// Visibility must be recomputed on the first frame during this turn
		unit.visibilityDirty = true;
	}
}

void CUnitManager::Interpolate(u32 turn, float frameTime, float frameOffset)
{
	PROFILE3("CUnitManager::Interpolate");

	++m_FrameNumber;
	m_FrameOffset = frameOffset;

	// TODO: we shouldn't update all the animations etc for units that are off-screen
	// (but need to be careful about e.g. sounds triggered by animations of off-screen
	// units)
	for (size_t i = 0; i < m_UnitsData.size(); i++)
	{
		SUnit& unit = m_UnitsData[i];
		if (unit.actor)
			unit.actor->UpdateModel(frameTime);
	}
	m_Turn = turn;

	m_DebugSpheres.clear();
	if (m_EnableDebugOverlays)
	{
		for (size_t i = 0; i < m_UnitsData.size(); i++)
		{
			SUnit& unit = m_UnitsData[i];
			if (!(unit.actor && unit.inWorld))
				continue;

			SOverlaySphere sphere;
			sphere.m_Center = unit.sweptBounds.GetCenter();
			sphere.m_Radius = unit.sweptBounds.GetRadius();
			if (unit.culled)
				sphere.m_Color = CColor(1.0f, 0.5f, 0.5f, 0.5f);
			else
				sphere.m_Color = CColor(0.5f, 0.5f, 1.0f, 0.5f);

			m_DebugSpheres.push_back(sphere);
		}
	}
}

void CUnitManager::RenderSubmit(SceneCollector& collector, const CFrustum& frustum, bool culling)
{
	// TODO: need a coarse culling pass based on some kind of spatial data
	// structure - that's the main point of this design. Once we've got a
	// rough list of possibly-visible units, then we can do the more precise
	// culling. (And once it's cheap enough, we can do multiple culling passes
	// per frame - one for shadow generation, one for water reflections, etc.)

	PROFILE3("CUnitManager::RenderSubmit");

	for (size_t i = 0; i < m_UnitsData.size(); ++i)
	{
		SUnit& unit = m_UnitsData[i];

		unit.culled = true;

		if (!unit.actor)
			continue;

		if (unit.visibilityDirty)
			UpdateVisibility(unit);

		if (unit.visibility == LosVisibility::HIDDEN)
			continue;

		if (!g_AtlasGameLoop->running && !g_RenderingOptions.GetRenderActors() && (unit.flags & ACTOR_ONLY))
			continue;

		if (!g_AtlasGameLoop->running && (unit.flags & VISIBLE_IN_ATLAS_ONLY))
			continue;

		if (culling && !frustum.IsSphereVisible(unit.sweptBounds.GetCenter(), unit.sweptBounds.GetRadius()))
			continue;

		unit.culled = false;

		CModelAbstract& unitModel = unit.actor->GetModel();

		if (unit.lastTransformFrame != m_FrameNumber)
		{
			CMatrix3D transform(GetInterpolatedTransform(unit, m_FrameOffset));

			unitModel.SetTransform(transform);

			unit.lastTransformFrame = m_FrameNumber;
		}

		if (culling && !frustum.IsBoxVisible(unitModel.GetWorldBoundsRec()))
			continue;

		collector.SubmitRecursive(&unitModel);
	}

	//for (size_t i = 0; i < m_DebugSpheres.size(); ++i)
		//collector.Submit(&m_DebugSpheres[i]);
}

void CUnitManager::UpdateVisibility(SUnit& unit) const
{
	unit.visibility = LosVisibility::VISIBLE;
	/*
	if (unit.inWorld)
	{
		// The 'always visible' flag means we should always render the unit
		// (regardless of whether the LOS system thinks it's visible)
		CmpPtr<ICmpVisibility> cmpVisibility(unit.entity);
		if (cmpVisibility && cmpVisibility->GetAlwaysVisible())
			unit.visibility = LosVisibility::VISIBLE;
		else
		{
			CmpPtr<ICmpRangeManager> cmpRangeManager(GetSystemEntity());
			unit.visibility = cmpRangeManager->GetLosVisibility(unit.entity,
																GetSimContext().GetCurrentDisplayedPlayer());
		}
	}
	else
		unit.visibility = LosVisibility::HIDDEN;

	// Change the visibility of the visual actor's selectable if it has one.
	CmpPtr<ICmpSelectable> cmpSelectable(unit.entity);
	if (cmpSelectable)
		cmpSelectable->SetVisibility(unit.visibility != LosVisibility::HIDDEN);

	unit.visibilityDirty = false;
	*/
}

CMatrix3D CUnitManager::GetInterpolatedTransform(SUnit& unit, float frameOffset) const
{
	/*
	if (m_TurretParent != INVALID_ENTITY)
	{
		CmpPtr<ICmpPosition> cmpPosition(GetSimContext(), m_TurretParent);
		if (!cmpPosition)
		{
			LOGERROR("Turret with parent without position component");
			CMatrix3D m;
			m.SetIdentity();
			return m;
		}
		if (!cmpPosition->IsInWorld())
		{
			LOGERROR("CCmpPosition::GetInterpolatedTransform called on turret entity when IsInWorld is false");
			CMatrix3D m;
			m.SetIdentity();
			return m;
		}
		else
		{
			CMatrix3D parentTransformMatrix = cmpPosition->GetInterpolatedTransform(frameOffset);
			CMatrix3D ownTransformation = CMatrix3D();
			ownTransformation.SetYRotation(m_InterpolatedRotY);
			ownTransformation.Translate(-m_TurretPosition.X.ToFloat(), m_TurretPosition.Y.ToFloat(), -m_TurretPosition.Z.ToFloat());
			return parentTransformMatrix * ownTransformation;
		}
	}
	*/
	if (!unit.inWorld)
	{
		LOGERROR("CCmpPosition::GetInterpolatedTransform called on entity when IsInWorld is false");
		CMatrix3D m;
		m.SetIdentity();
		return m;
	}

	float x, z, rotY = 0.f;
	x = ::Interpolate(unit.pos0.X, unit.pos1.X, frameOffset);
	z = ::Interpolate(unit.pos0.Z, unit.pos1.Z, frameOffset);

	//rotY = m_InterpolatedRotY;

	float baseY = 0;
	/*
	if (m_RelativeToGround)
	{
		CmpPtr<ICmpTerrain> cmpTerrain(GetSystemEntity());
		if (cmpTerrain)
			baseY = cmpTerrain->GetExactGroundLevel(x, z);

		if (m_Floating || m_ActorFloating)
		{
			CmpPtr<ICmpWaterManager> cmpWaterManager(GetSystemEntity());
			if (cmpWaterManager)
				baseY = std::max(baseY, cmpWaterManager->GetExactWaterLevel(x, z) - m_FloatDepth.ToFloat());
		}
	}*/

	float y = baseY + ::Interpolate(unit.pos0.Y, unit.pos1.Y, frameOffset);

	CMatrix3D m;
	m.SetIdentity();
	// linear interpolation is good enough (for RotX/Z).
	// As you always stay close to zero angle.
	//m.SetXRotation(::Interpolate(m_LastInterpolatedRotX, m_InterpolatedRotX, frameOffset));
	//m.RotateZ(::Interpolate(m_LastInterpolatedRotZ, m_InterpolatedRotZ, frameOffset));

	CVector3D pos(x, y, z);

	//pos.Y += GetConstructionProgressOffset(pos);

	m.RotateY(rotY + (float)M_PI);
	m.Translate(pos);

	return m;
}
