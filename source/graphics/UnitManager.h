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

#ifndef INCLUDED_UNITMANAGER
#define INCLUDED_UNITMANAGER

#include "graphics/Overlay.h"
#include "maths/BoundingSphere.h"
#include "ps/CStrForward.h"
#include "renderer/Scene.h"
#include "simulation2/components/ICmpRangeManager.h"

#include <vector>
#include <set>

class CUnit;
class CObjectManager;

class CMatrix3D;

///////////////////////////////////////////////////////////////////////////////
// CUnitManager: simple container class holding all units within the world
class CUnitManager
{
public:

	enum
	{
		ACTOR_ONLY = 1 << 0,
		VISIBLE_IN_ATLAS_ONLY = 1 << 1,
	};

	/**
	 * External identifiers for models.
	 * (This is a struct rather than a raw u32 for type-safety.)
	 */
	struct tag_t
	{
		tag_t() : n(0) {}
		explicit tag_t(u32 n) : n(n) {}
		bool valid() const { return n != 0; }

		u32 n;
	};

	struct SUnit
	{
		entity_id_t entity_id;

		CUnit* actor;

		int flags;

		/**
		 * m_FrameNumber from when the model's transform was last updated.
		 * This is used to avoid recomputing it multiple times per frame
		 * if a model is visible in multiple cull groups.
		 */
		int lastTransformFrame;

		/**
		 * Worst-case bounding shape, relative to position. Needs to account
		 * for all possible animations, orientations, etc.
		 */
		CBoundingSphere boundsApprox;

		/**
		 * Cached LOS visibility status.
		 */
		LosVisibility visibility;
		bool visibilityDirty;

		/**
		 * Whether the unit has a valid position. If false, pos0 and pos1
		 * are meaningless.
		 */
		bool inWorld;

		/**
		 * World-space positions to interpolate between.
		 */
		CVector3D pos0;
		CVector3D pos1;

		/**
		 * Bounds encompassing the unit's bounds when it is anywhere between
		 * pos0 and pos1.
		 */
		CBoundingSphere sweptBounds;

		/**
		 * For debug overlay.
		 */
		bool culled;
	};

	void TurnStart();
	void Interpolate(u32 turn, float frameTime, float frameOffset);
	void RenderSubmit(SceneCollector& collector, const CFrustum& frustum, bool culling);

	void UpdateVisibility(SUnit& unit) const;

	SUnit* LookupUnit(tag_t tag);
	void RecomputeSweptBounds(SUnit* unit);

	tag_t AddUnit(CEntityHandle entity, CUnit* unit, const CBoundingSphere& boundsApprox, int flags);

	void RemoveUnit(tag_t tag);

	void UpdateUnit(tag_t tag, CUnit* unit, const CBoundingSphere& boundsApprox);

	void UpdateUnitPos(tag_t tag, bool inWorld, const CVector3D& pos0, const CVector3D& pos1);

	CMatrix3D GetInterpolatedTransform(SUnit& unit, float frameOffset) const;
	/**
	 * Return a list of visual entities along with their center point.
	 * Visual means they have an associated actor and a visual component,
	 * but they could still be hiden in the fog of war for a specific player,
	 * for example.
	 * NOTE: It's generally faster to do a lot of ray intersection tests than
	 * querying a lot of entities for component interfaces and doing these types
	 * of tests first.
	 */
	void PickAllEntitiesAtPoint(std::vector<std::pair<CEntityHandle, CVector3D> >& outEntities,
		const CVector3D& origin, const CVector3D& dir,
		bool allowEditorSelectables) const;

	/**
	 * Returns the frame offset from the last Interpolate message.
	 */
	float GetFrameOffset() const;

	/**
	 * Toggle the rendering of debug info.
	 */
	void SetDebugOverlay(bool enabled);

public:
	// constructor, destructor
	CUnitManager();
	~CUnitManager();

	// add given unit to world
	void AddUnit(CUnit* unit);
	// remove given unit from world, but don't delete it
	void RemoveUnit(CUnit* unit);
	// remove given unit from world and delete it
	void DeleteUnit(CUnit* unit);
	// remove and delete all units
	void DeleteAll();

	// creates a new unit and adds it to the world
	CUnit* CreateUnit(const CStrW& actorName, uint32_t seed);

	// return the units
	const std::vector<CUnit*>& GetUnits() const { return m_Units; }

	void SetObjectManager(CObjectManager& objectManager) { m_ObjectManager = &objectManager; }

	/**
	 * Mark a specific region of the terrain as dirty.
	 * Coordinates are in terrain tiles, lower inclusive, upper exclusive.
	 */
	void MakeTerrainDirty(ssize_t i0, ssize_t j0, ssize_t i1, ssize_t j1, int dirtyFlags);

private:
	// list of all known units
	std::vector<CUnit*> m_Units;
	// graphical object manager; may be NULL if not set up
	CObjectManager* m_ObjectManager;

	std::vector<SUnit> m_UnitsData;
	std::vector<tag_t> m_UnitTagsFree;

	u32 m_Turn;
	int m_FrameNumber;
	float m_FrameOffset;

	bool m_EnableDebugOverlays;
	std::vector<SOverlaySphere> m_DebugSpheres;
};

#endif
