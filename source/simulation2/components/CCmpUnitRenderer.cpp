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

#include "simulation2/system/Component.h"
#include "ICmpUnitRenderer.h"

#include "simulation2/MessageTypes.h"

#include "ICmpPosition.h"
#include "ICmpRangeManager.h"
#include "ICmpSelectable.h"
#include "ICmpVisibility.h"
#include "ICmpVisual.h"

#include "graphics/ModelAbstract.h"
#include "graphics/ObjectEntry.h"
#include "graphics/Overlay.h"
#include "graphics/Unit.h"
#include "graphics/UnitManager.h"
#include "maths/BoundingSphere.h"
#include "maths/Frustum.h"
#include "maths/Matrix3D.h"
#include "ps/GameSetup/Config.h"
#include "ps/Profile.h"
#include "renderer/RenderingOptions.h"
#include "renderer/Scene.h"

#include "tools/atlas/GameInterface/GameLoop.h"

/**
 * Efficiently(ish) renders all the units in the world.
 *
 * The class maintains a list of all units that currently exist, and the data
 * needed for frustum-culling them. To minimise the amount of work done per
 * frame (despite a unit's interpolated position changing every frame), the
 * culling data is only updated once per turn: we store the position at the
 * start of the turn, and the position at the end of the turn, and assume the
 * unit might be anywhere between those two points (linearly).
 *
 * (Note this is a slightly invalid assumption: units don't always move linearly,
 * since their interpolated position depends on terrain and water. But over a
 * single turn it's probably going to be a good enough approximation, and will
 * only break for units that both start and end the turn off-screen.)
 *
 * We want to ignore rotation entirely, since it's a complex function of
 * interpolated position and terrain. So we store a bounding sphere, which
 * is rotation-independent, instead of a bounding box.
 */
class CCmpUnitRenderer final : public ICmpUnitRenderer
{
public:
	struct SUnit
	{
		CEntityHandle entity;

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

	std::vector<SUnit> m_Units;
	std::vector<tag_t> m_UnitTagsFree;

	int m_FrameNumber;
	float m_FrameOffset;

	bool m_EnableDebugOverlays;
	std::vector<SOverlaySphere> m_DebugSpheres;

	static void ClassInit(CComponentManager& componentManager)
	{
		componentManager.SubscribeToMessageType(MT_TurnStart);
	}

	DEFAULT_COMPONENT_ALLOCATOR(UnitRenderer)

	static std::string GetSchema()
	{
		return "<a:component type='system'/><empty/>";
	}

	void Init(const CParamNode& UNUSED(paramNode)) override
	{
		m_FrameNumber = 0;
		m_FrameOffset = 0.0f;
		m_EnableDebugOverlays = false;
	}

	void Deinit() override
	{
	}

	void Serialize(ISerializer& UNUSED(serialize)) override
	{
	}

	void Deserialize(const CParamNode& paramNode, IDeserializer& UNUSED(deserialize)) override
	{
		Init(paramNode);
	}

	void HandleMessage(const CMessage& msg, bool UNUSED(global)) override
	{
		switch (msg.GetType())
		{
		case MT_TurnStart:
		{
			TurnStart();
			break;
		}
		}
	}

	SUnit* LookupUnit(tag_t tag)
	{
		if (tag.n < 1 || tag.n - 1 >= m_Units.size())
			return NULL;
		return &m_Units[tag.n - 1];
	}

	tag_t AddUnit(CEntityHandle entity, CUnit* actor, const CBoundingSphere& boundsApprox, int flags) override
	{
		GetSimContext().GetUnitManager().AddUnit(entity, actor, boundsApprox, flags);

		ENSURE(actor != NULL);

		tag_t tag;
		if (!m_UnitTagsFree.empty())
		{
			tag = m_UnitTagsFree.back();
			m_UnitTagsFree.pop_back();
		}
		else
		{
			m_Units.push_back(SUnit());
			tag.n = m_Units.size();
		}

		SUnit* unit = LookupUnit(tag);
		unit->entity = entity;
		unit->actor = actor;
		unit->lastTransformFrame = -1;
		unit->flags = flags;
		unit->boundsApprox = boundsApprox;
		unit->inWorld = false;
		unit->visibilityDirty = true;
		unit->pos0 = unit->pos1 = CVector3D();

		return tag;
	}

	void RemoveUnit(tag_t tag) override
	{
		GetSimContext().GetUnitManager().RemoveUnit(*(CUnitManager::tag_t*)((void*)(&tag)));
		SUnit* unit = LookupUnit(tag);
		unit->actor = NULL;
		unit->inWorld = false;
		m_UnitTagsFree.push_back(tag);
	}

	void RecomputeSweptBounds(SUnit* unit)
	{
		// Compute the bounding sphere of the capsule formed by
		// sweeping boundsApprox from pos0 to pos1
		CVector3D mid = (unit->pos0 + unit->pos1) * 0.5f + unit->boundsApprox.GetCenter();
		float radius = (unit->pos1 - unit->pos0).Length() * 0.5f + unit->boundsApprox.GetRadius();
		unit->sweptBounds = CBoundingSphere(mid, radius);
	}

	void UpdateUnit(tag_t tag, CUnit* actor, const CBoundingSphere& boundsApprox) override
	{
		GetSimContext().GetUnitManager().UpdateUnit(*(CUnitManager::tag_t*)((void*)(&tag)), actor, boundsApprox);
		SUnit* unit = LookupUnit(tag);
		unit->actor = actor;
		unit->boundsApprox = boundsApprox;
		RecomputeSweptBounds(unit);
	}

	void UpdateUnitPos(tag_t tag, bool inWorld, const CVector3D& pos0, const CVector3D& pos1) override
	{
		GetSimContext().GetUnitManager().UpdateUnitPos(*(CUnitManager::tag_t*)((void*)(&tag)), inWorld, pos0, pos1);
		SUnit* unit = LookupUnit(tag);
		unit->inWorld = inWorld;
		unit->pos0 = pos0;
		unit->pos1 = pos1;
		unit->visibilityDirty = true;
		RecomputeSweptBounds(unit);
	}

	void TurnStart() {
	}

	float GetFrameOffset() const override
	{
		return m_FrameOffset;
	}

	void SetDebugOverlay(bool enabled) override
	{
		m_EnableDebugOverlays = enabled;
	}

	void PickAllEntitiesAtPoint(std::vector<std::pair<CEntityHandle, CVector3D> >& outEntities, const CVector3D& origin, const CVector3D& dir, bool allowEditorSelectables) const override
	{
		// First, make a rough test with the worst-case bounding boxes to pick all
		// entities/models that could possibly be hit by the ray.
		std::vector<const SUnit*> candidates;
		for (const SUnit& unit : m_Units)
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
	}
};

REGISTER_COMPONENT_TYPE(UnitRenderer)
