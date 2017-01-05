/* Copyright (C) 2016 Wildfire Games.
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
#include "ICmpUnitMotion.h"

#include "simulation2/components/ICmpObstruction.h"
#include "simulation2/components/ICmpObstructionManager.h"
#include "simulation2/components/ICmpOwnership.h"
#include "simulation2/components/ICmpPosition.h"
#include "simulation2/components/ICmpPathfinder.h"
#include "simulation2/components/ICmpRangeManager.h"
#include "simulation2/components/ICmpValueModificationManager.h"
#include "simulation2/components/ICmpVisual.h"
#include "simulation2/helpers/Geometry.h"
#include "simulation2/helpers/Render.h"
#include "simulation2/MessageTypes.h"
#include "simulation2/serialization/SerializeTemplates.h"

#include "graphics/Overlay.h"
#include "graphics/Terrain.h"
#include "maths/FixedVector2D.h"
#include "ps/CLogger.h"
#include "ps/Profile.h"
#include "renderer/Scene.h"

// For debugging; units will start going straight to the target
// instead of calling the pathfinder
#define DISABLE_PATHFINDER 0

/**
 * When advancing along the long path, and picking a new waypoint to move
 * towards, we'll pick one that's up to this far from the unit's current
 * position (to minimise the effects of grid-constrained movement)
 */
static const entity_pos_t WAYPOINT_ADVANCE_MAX = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*8);

/**
 * Min/Max range to restrict short path queries to. (Larger ranges are slower,
 * smaller ranges might miss some legitimate routes around large obstacles.)
 */
static const entity_pos_t SHORT_PATH_MIN_SEARCH_RANGE = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*2);
static const entity_pos_t SHORT_PATH_MAX_SEARCH_RANGE = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*9);

/**
 * Minimum distance to goal for a long path request
 */
static const entity_pos_t LONG_PATH_MIN_DIST = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*4);

/**
 * When short-pathing, and the short-range pathfinder failed to return a path,
 * Assume we are at destination if we are closer than this distance to the target
 * And we have no target entity.
 * This is somewhat arbitrary, but setting a too big distance means units might lose sight of their end goal too much;
 */
static const entity_pos_t SHORT_PATH_GOAL_RADIUS = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*2);

/**
 * If we are this close to our target entity/point, then think about heading
 * for it in a straight line instead of pathfinding.
 */
static const entity_pos_t DIRECT_PATH_RANGE = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*4);

/**
 * If we're following a target entity,
 * we will recompute our path if the target has moved
 * more than this distance from where we last pathed to.
 */
static const entity_pos_t CHECK_TARGET_MOVEMENT_MIN_DELTA = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*4);

/**
 * If we're following as part of a formation,
 * but can't move to our assigned target point in a straight line,
 * we will recompute our path if the target has moved
 * more than this distance from where we last pathed to.
 */
static const entity_pos_t CHECK_TARGET_MOVEMENT_MIN_DELTA_FORMATION = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*1);

/**
 * If we're following something but it's more than this distance away along
 * our path, then don't bother trying to repath regardless of how much it has
 * moved, until we get this close to the end of our old path.
 */
static const entity_pos_t CHECK_TARGET_MOVEMENT_AT_MAX_DIST = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*16);

/**
 * If we're following something and the angle between the (straight-line) directions to its previous target
 * position and its present target position is greater than a given angle, recompute the path even far away
 * (i.e. even if CHECK_TARGET_MOVEMENT_AT_MAX_DIST condition is not fulfilled). The actual check is done
 * on the cosine of this angle, with a PI/6 angle.
 */
static const fixed CHECK_TARGET_MOVEMENT_MIN_COS = fixed::FromInt(866)/1000;

/**
 * See unitmotion logic for details. Higher means units will retry more often before potentially failing.
 */
static const size_t MAX_PATH_REATTEMPS = 8;

static const CColor OVERLAY_COLOR_LONG_PATH(1, 1, 1, 1);
static const CColor OVERLAY_COLOR_SHORT_PATH(1, 0, 0, 1);

class CCmpUnitMotion : public ICmpUnitMotion
{
private:
	struct SMotionGoal
	{
	private:
		bool m_Valid = false;

		entity_pos_t m_TargetMinRange;
		entity_pos_t m_TargetMaxRange;

		entity_id_t m_TargetEntity;
		// pathfinder-compliant goal.
		PathGoal m_Goal;
	public:
		SMotionGoal() : m_Valid(false) {};

		SMotionGoal(PathGoal& goal, entity_pos_t minRange, entity_pos_t maxRange)
		{
			m_TargetEntity = INVALID_ENTITY;

			m_TargetMinRange = minRange;
			m_TargetMaxRange = maxRange;

			m_Goal = goal;
			m_Valid = true;
		}

		SMotionGoal(const CSimContext& context, entity_id_t target, PathGoal& goal, entity_pos_t minRange, entity_pos_t maxRange)
		{
			m_TargetEntity = target;
			m_TargetMinRange = minRange;
			m_TargetMaxRange = maxRange;

			m_Goal = goal;
			m_Valid = true;

			UpdateTargetPosition(context);
		}

		template<typename S>
		void SerializeCommon(S& serialize)
		{
			serialize.Bool("valid", m_Valid);

			serialize.NumberFixed_Unbounded("target min range", m_TargetMinRange);
			serialize.NumberFixed_Unbounded("target max range", m_TargetMaxRange);

			serialize.NumberU32_Unbounded("target entity", m_TargetEntity);

			SerializeGoal()(serialize, "goal", m_Goal);
		}

		const PathGoal& Goal() const { return m_Goal; };

		bool TargetIsEntity() const { return m_TargetEntity != INVALID_ENTITY; }
		entity_id_t GetEntity() const { return m_TargetEntity; }

		bool Valid() const { return m_Valid; }
		void Clear() { m_Valid = false; }

		entity_pos_t MinRange() const { return m_TargetMinRange; };
		entity_pos_t MaxRange() const { return m_TargetMaxRange; };

		CFixedVector2D Pos() const { return CFixedVector2D(m_Goal.x, m_Goal.z); }
		entity_pos_t X() const { return m_Goal.x; }
		entity_pos_t Z() const { return m_Goal.z; }

		void UpdateTargetPosition(const CSimContext& context)
		{
			if (!TargetIsEntity())
				return;

			CmpPtr<ICmpPosition> cmpPosition(context, m_TargetEntity);
			if (!cmpPosition || !cmpPosition->IsInWorld())
				return;

			m_Goal.x = cmpPosition->GetPosition2D().X;
			m_Goal.z = cmpPosition->GetPosition2D().Y;
		}

		bool IsNotAPoint() const
		{
			return m_TargetMaxRange > fixed::Zero() || m_Goal.type != PathGoal::POINT;
		}
	};

public:
	static void ClassInit(CComponentManager& componentManager)
	{
		componentManager.SubscribeToMessageType(MT_Update_MotionUnit);
		componentManager.SubscribeToMessageType(MT_PathResult);
		componentManager.SubscribeToMessageType(MT_OwnershipChanged);
		componentManager.SubscribeToMessageType(MT_ValueModification);
		componentManager.SubscribeToMessageType(MT_Deserialized);
	}

	DEFAULT_COMPONENT_ALLOCATOR(UnitMotion)

	bool m_DebugOverlayEnabled;
	std::vector<SOverlayLine> m_DebugOverlayLongPathLines;
	std::vector<SOverlayLine> m_DebugOverlayShortPathLines;

	// Template state, never changed after init.
	fixed m_TemplateWalkSpeed, m_TemplateTopSpeedRatio;
	pass_class_t m_PassClass;
	std::string m_PassClassName;
	entity_pos_t m_Clearance;

	// cached for efficiency
	fixed m_TechModifiedWalkSpeed, m_TechModifiedTopSpeedRatio;

	// TARGET
	// As long as we have a valid destination, the unit is seen as trying to move
	// It may not be actually moving for a variety of reasons (no path, blocked path)…
	// this is our final destination
	SMotionGoal m_Destination;
	// this is a "temporary" destination. Most of the time it will be the same as m_Destination,
	// but it doesn't have to be.
	// Can be used to temporarily re-route or pause a unit from a given component, for whatever reason.
	// Will also be used whenever I implemented step-by-step long paths using the hierarchical pathfinder.
	SMotionGoal m_CurrentGoal;

	// MOTION PLANNING
	// We will abort if we are stuck after X tries.
	u8 m_AbortIfStuck;
	// turn towards our target at the end
	bool m_FacePointAfterMove;
	// actual unit speed, after technology and ratio
	fixed m_Speed;
	// cached for convenience
	fixed m_SpeedRatio;

	// asynchronous request ID we're waiting for, or 0 if none
	u32 m_ExpectedPathTicket;

	// Currently active paths (storing waypoints in reverse order).
	// The last item in each path is the point we're currently heading towards.
	WaypointPath m_Path;
	// used for the short pathfinder, incremented on each unsuccessful try.
	u8 m_Tries;
	// Turns to wait before a certain action.
	u8 m_WaitingTurns;
	// if we actually started moving at some point.
	bool m_StartedMoving;

	// Speed over the last turn
	// cached so we can tell the visual actor when it changes
	fixed m_ActualSpeed;

	static std::string GetSchema()
	{
		return
			"<a:help>Provides the unit with the ability to move around the world by itself.</a:help>"
			"<a:example>"
				"<WalkSpeed>7.0</WalkSpeed>"
				"<PassabilityClass>default</PassabilityClass>"
			"</a:example>"
			"<element name='FormationController'>"
				"<data type='boolean'/>"
			"</element>"
			"<element name='WalkSpeed' a:help='Basic movement speed (in metres per second)'>"
				"<ref name='positiveDecimal'/>"
			"</element>"
			"<optional>"
				"<element name='RunMultiplier' a:help='How much faster the unit goes when running (as a multiple of walk speed)'>"
					"<ref name='positiveDecimal'/>"
				"</element>"
			"</optional>"
			"<element name='PassabilityClass' a:help='Identifies the terrain passability class (values are defined in special/pathfinder.xml)'>"
				"<text/>"
			"</element>";
	}

	virtual void Init(const CParamNode& paramNode)
	{
		m_FacePointAfterMove = true;

		m_TechModifiedWalkSpeed = m_TemplateWalkSpeed = m_Speed = paramNode.GetChild("WalkSpeed").ToFixed();
		m_ActualSpeed = fixed::Zero();
		m_SpeedRatio = fixed::FromInt(1);

		m_TechModifiedTopSpeedRatio = m_TemplateTopSpeedRatio = fixed::FromInt(1);
		if (paramNode.GetChild("RunMultiplier").IsOk())
			m_TechModifiedTopSpeedRatio = m_TemplateTopSpeedRatio = paramNode.GetChild("RunMultiplier").ToFixed();

		CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
		if (cmpPathfinder)
		{
			m_PassClassName = paramNode.GetChild("PassabilityClass").ToUTF8();
			m_PassClass = cmpPathfinder->GetPassabilityClass(m_PassClassName);
			m_Clearance = cmpPathfinder->GetClearance(m_PassClass);

			CmpPtr<ICmpObstruction> cmpObstruction(GetEntityHandle());
			if (cmpObstruction)
				cmpObstruction->SetUnitClearance(m_Clearance);
		}

		m_ExpectedPathTicket = 0;

		m_Tries = 0;
		m_WaitingTurns = 0;

		m_DebugOverlayEnabled = false;
		m_AbortIfStuck = 0;
	}

	virtual void Deinit()
	{
	}

	template<typename S>
	void SerializeCommon(S& serialize)
	{
		serialize.NumberU8("abort if stuck", m_AbortIfStuck, 0, 255);
		serialize.Bool("face point after move", m_FacePointAfterMove);
		serialize.NumberFixed_Unbounded("speed", m_Speed);
		serialize.NumberFixed_Unbounded("speed ratio", m_SpeedRatio);

		serialize.NumberU32_Unbounded("ticket", m_ExpectedPathTicket);

		SerializeVector<SerializeWaypoint>()(serialize, "path", m_Path.m_Waypoints);

		serialize.NumberU8("tries", m_Tries, 0, 255);
		serialize.NumberU8("waiting turns", m_WaitingTurns, 0, 255);

		serialize.Bool("started moving", m_StartedMoving);

		// strictly speaking this doesn't need to be serialized since it's graphics-only, but it's nicer to.
		serialize.NumberFixed_Unbounded("actual speed", m_ActualSpeed);

		m_Destination.SerializeCommon(serialize);
		m_CurrentGoal.SerializeCommon(serialize);
	}

	virtual void Serialize(ISerializer& serialize)
	{
		SerializeCommon(serialize);
	}

	virtual void Deserialize(const CParamNode& paramNode, IDeserializer& deserialize)
	{
		Init(paramNode);

		SerializeCommon(deserialize);

		CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
		if (cmpPathfinder)
			m_PassClass = cmpPathfinder->GetPassabilityClass(m_PassClassName);
	}

	virtual void HandleMessage(const CMessage& msg, bool UNUSED(global))
	{
		switch (msg.GetType())
		{
		case MT_Update_MotionUnit:
		{
			fixed dt = static_cast<const CMessageUpdate_MotionUnit&> (msg).turnLength;
			Move(dt);
			break;
		}
		case MT_RenderSubmit:
		{
			PROFILE("UnitMotion::RenderSubmit");
			const CMessageRenderSubmit& msgData = static_cast<const CMessageRenderSubmit&> (msg);
			RenderSubmit(msgData.collector);
			break;
		}
		case MT_PathResult:
		{
			const CMessagePathResult& msgData = static_cast<const CMessagePathResult&> (msg);
			PathResult(msgData.ticket, msgData.path);
			break;
		}
		case MT_ValueModification:
		{
			const CMessageValueModification& msgData = static_cast<const CMessageValueModification&> (msg);
			if (msgData.component != L"UnitMotion")
				break;
		}
		// fall-through
		case MT_Deserialized:
		{
			// tell the visual actor our speed.
			// don't call setactualspeed since the if check will return immediately.
			CmpPtr<ICmpVisual> cmpVisualActor(GetEntityHandle());
			if (cmpVisualActor)
				cmpVisualActor->SetMovingSpeed(m_ActualSpeed);
		}
		case MT_OwnershipChanged:
		{
			CmpPtr<ICmpValueModificationManager> cmpValueModificationManager(GetSystemEntity());
			if (!cmpValueModificationManager)
				break;

			m_TechModifiedWalkSpeed = cmpValueModificationManager->ApplyModifications(L"UnitMotion/WalkSpeed", m_TemplateWalkSpeed, GetEntityId());
			m_TechModifiedTopSpeedRatio = cmpValueModificationManager->ApplyModifications(L"UnitMotion/RunMultiplier", m_TemplateTopSpeedRatio, GetEntityId());

			m_Speed = m_SpeedRatio.Multiply(GetBaseSpeed());

			break;
		}
		}
	}

	void UpdateMessageSubscriptions()
	{
		bool needRender = m_DebugOverlayEnabled;
		GetSimContext().GetComponentManager().DynamicSubscriptionNonsync(MT_RenderSubmit, this, needRender);
	}

	virtual bool IsActuallyMoving()
	{
		return m_StartedMoving;
	}

	virtual bool IsTryingToMove()
	{
		return m_Destination.Valid();
	}

	virtual fixed GetBaseSpeed()
	{
		return m_TechModifiedWalkSpeed;
	}

	virtual fixed GetSpeed()
	{
		return m_Speed;
	}

	virtual fixed GetSpeedRatio()
	{
		return m_SpeedRatio;
	}

	virtual fixed GetTopSpeedRatio()
	{
		return m_TechModifiedTopSpeedRatio;
	}

	virtual void SetSpeed(fixed ratio)
	{
		m_SpeedRatio = std::min(ratio, GetTopSpeedRatio());
		m_Speed = m_SpeedRatio.Multiply(GetBaseSpeed());
	}

	// convenience wrapper
	void SetActualSpeed(fixed newRealSpeed)
	{
		if (m_ActualSpeed == newRealSpeed)
			return;

		m_ActualSpeed = newRealSpeed;
		CmpPtr<ICmpVisual> cmpVisualActor(GetEntityHandle());
		if (cmpVisualActor)
			cmpVisualActor->SetMovingSpeed(m_ActualSpeed);
	}

	virtual pass_class_t GetPassabilityClass()
	{
		return m_PassClass;
	}

	virtual std::string GetPassabilityClassName()
	{
		return m_PassClassName;
	}

	virtual void SetPassabilityClassName(std::string passClassName)
	{
		m_PassClassName = passClassName;
		CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
		if (cmpPathfinder)
			m_PassClass = cmpPathfinder->GetPassabilityClass(passClassName);
	}

	virtual void SetFacePointAfterMove(bool facePointAfterMove)
	{
		m_FacePointAfterMove = facePointAfterMove;
	}

	virtual void SetDebugOverlay(bool enabled)
	{
		m_DebugOverlayEnabled = enabled;
		UpdateMessageSubscriptions();
	}

	virtual entity_pos_t GetUnitClearance()
	{
		return m_Clearance;
	}

	virtual bool MoveToPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange);
	virtual bool IsInPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange);
	virtual bool MoveToTargetRange(entity_id_t target, entity_pos_t minRange, entity_pos_t maxRange);
	virtual bool IsInTargetRange(entity_id_t target, entity_pos_t minRange, entity_pos_t maxRange);

	virtual void FaceTowardsPoint(entity_pos_t x, entity_pos_t z);

	virtual void SetAbortIfStuck(u8 shouldAbort)
	{
		m_AbortIfStuck = shouldAbort;
	}

	void StartMoving()
	{
		m_StartedMoving = true;

		CmpPtr<ICmpObstruction> cmpObstruction(GetEntityHandle());
		if (cmpObstruction)
			cmpObstruction->SetMovingFlag(true);
	}

	virtual void StopMoving()
	{
		m_StartedMoving = false;

		CmpPtr<ICmpObstruction> cmpObstruction(GetEntityHandle());
		if (cmpObstruction)
			cmpObstruction->SetMovingFlag(false);
	}

	virtual void DiscardMove()
	{
		StopMoving();

		m_Tries = 0;
		m_WaitingTurns = 0;

		m_ExpectedPathTicket = 0;
		m_Path.m_Waypoints.clear();

		m_CurrentGoal.Clear();
		m_Destination.Clear();
	}

	void MoveWillFail()
	{

	}

	virtual bool HasValidPath()
	{
		return !m_Path.m_Waypoints.empty();
	}

private:
/*
 TODO: reimplement
	bool IsFormationMember() const
	{
		return m_State == STATE_FORMATIONMEMBER_PATH;
	}
*/
	entity_id_t GetGroup() const
	{
		//return IsFormationMember() ? m_TargetEntity : GetEntityId();
		return GetEntityId();
	}

	bool MoveToPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange, entity_id_t target);

	/**
	 * Handle the result of an asynchronous path query.
	 */
	void PathResult(u32 ticket, const WaypointPath& path);

	/**
	 * Do the per-turn movement and other updates.
	 */
	void Move(fixed dt);

	/**
	 * Returns whether we are close enough to the target to assume it's a good enough
	 * position to stop.
	 */
	bool ShouldConsiderOurselvesAtDestination();

	/**
	 * Returns whether the length of the given path, plus the distance from
	 * 'from' to the first waypoints, it shorter than minDistance.
	 */
	bool PathIsShort(const WaypointPath& path, const CFixedVector2D& from, entity_pos_t minDistance) const;

	/**
	 * Rotate to face towards the target point, given the current pos
	 */
	void FaceTowardsPointFromPos(const CFixedVector2D& pos, entity_pos_t x, entity_pos_t z);

	/**
	 * Returns an appropriate obstruction filter for use with path requests.
	 * noTarget is true only when used inside tryGoingStraightToTargetEntity,
	 * in which case we do not want the target obstruction otherwise it would always fail
	 */
	ControlGroupMovementObstructionFilter GetObstructionFilter(bool noTarget = false) const;

	/**
	 * Dump current paths and request a new one.
	 * Might go in a straight line immediately, or might start an asynchronous
	 * path request.
	 */
	void RequestNewPath();

	/**
	 * Start an asynchronous long path query.
	 */
	void RequestLongPath(const CFixedVector2D& from, const PathGoal& goal);

	/**
	 * Start an asynchronous short path query.
	 */
	void RequestShortPath(const CFixedVector2D& from, const PathGoal& goal, bool avoidMovingUnits);

	/**
	 * Convert a path into a renderable list of lines
	 */
	void RenderPath(const WaypointPath& path, std::vector<SOverlayLine>& lines, CColor color);

	void RenderSubmit(SceneCollector& collector);
};

REGISTER_COMPONENT_TYPE(UnitMotion)

void CCmpUnitMotion::PathResult(u32 ticket, const WaypointPath& path)
{
	// Ignore obsolete path requests
	if (ticket != m_ExpectedPathTicket)
		return;

	m_ExpectedPathTicket = 0; // we don't expect to get this result again

	if (!m_Destination.Valid())
		return;

	if (path.m_Waypoints.empty())
	{
		// no waypoints, path failed.
		// if we have some room, pop waypoint
		// TODO: this isn't particularly bright.
		if (!m_Path.m_Waypoints.empty())
			m_Path.m_Waypoints.pop_back();

		// we will then deal with this on the next Move() call.
		return;
	}

	// add to the top of our current waypoints
	m_Path.m_Waypoints.insert(m_Path.m_Waypoints.end(), path.m_Waypoints.begin(), path.m_Waypoints.end());
}

void CCmpUnitMotion::Move(fixed dt)
{
	PROFILE("Move");

	if (!IsTryingToMove())
	{
		SetActualSpeed(fixed::Zero());
		return;
	}

	// TODO: units will look at each other's position in an arbitrary order that must be the same for any simulation
	// In particular this means no threading. Maybe we should update this someday if it's possible.

	CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
	if (!cmpPathfinder)
		return;

	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (!cmpPosition || !cmpPosition->IsInWorld())
		return;

	CFixedVector2D initialPos = cmpPosition->GetPosition2D();

	// Check wether we are at our destination.
	// This must be done only at the beginning of a turn, if we do at the end of a turn (after a unit position has changed)
	// the unit's position will interpolate but the unit will already be doing the next thing, so it looks like it's gliding.
	if (ShouldConsiderOurselvesAtDestination())
	{
		// TODO: change this
		DiscardMove();
		return;
	}

	// TODO: here should go things such as:
	// - has our target moved enough that we should re-path?
	// end TODO

	// Keep track of the current unit's position during the update
	CFixedVector2D pos = initialPos;

	// Find the speed factor of the underlying terrain
	// (We only care about the tile we start on - it doesn't matter if we're moving
	// partially onto a much slower/faster tile)
	// TODO: Terrain-dependent speeds are not currently supported
	// TODO: note that this is also linked to pathfinding so maybe never supported
	// fixed terrainSpeed = fixed::FromInt(1);

	bool wasObstructed = false;

	// We want to move (at most) m_Speed*dt units from pos towards the next waypoint

	fixed timeLeft = dt;

	// TODO: I think this may be a little buggy if we want to compute it several times per turn.
	while (timeLeft > fixed::Zero())
	{
		// If we ran out of path, we have to stop
		if (!HasValidPath())
			break;

		CFixedVector2D target;
		target = CFixedVector2D(m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z);

		CFixedVector2D offset = target - pos;
		fixed offsetLength = offset.Length();
		// Work out how far we can travel in timeLeft
		fixed maxdist = m_Speed.Multiply(timeLeft);

		CFixedVector2D destination;
		if (offsetLength <= maxdist)
			destination = target;
		else
		{
			offset.Normalize(maxdist);
			destination = pos + offset;
		}

		// TODO: try moving as much as we can still?
		// TODO: get more information about what blocked us.
		if (cmpPathfinder->CheckMovement(GetObstructionFilter(), pos.X, pos.Y, destination.X, destination.Y, m_Clearance, m_PassClass))
		{
			pos = destination;

			timeLeft = (timeLeft.Multiply(m_Speed) - offsetLength) / m_Speed;

			if (destination == target)
				m_Path.m_Waypoints.pop_back();
			continue;
		}
		else
		{
			// Error - path was obstructed
			wasObstructed = true;
			break;
		}
	}

	if (!m_StartedMoving && wasObstructed)
		// If this is the turn we start moving, and we're already obstructed,
		// fail the move entirely to avoid weirdness.
		// TODO: figure out if this is actually necessary with the other changes
		pos = initialPos;

	// Update the Position component after our movement (if we actually moved anywhere)
	if (pos != initialPos)
	{
		CFixedVector2D offset = pos - initialPos;

		// tell other components and visual actor we are moving.
		if (!m_StartedMoving)
			StartMoving();

		// Calculate the mean speed over this past turn.
		// TODO: this is often just a little different from our actual top speed
		// so we end up changing the actual speed quite often, which is a little silly.
		SetActualSpeed(cmpPosition->GetDistanceTravelled() / dt);

		// Face towards the target
		entity_angle_t angle = atan2_approx(offset.X, offset.Y);
		cmpPosition->MoveAndTurnTo(pos.X,pos.Y, angle);

		if (!wasObstructed)
		{
			// everything is going smoothly, return.
			m_Tries = 0;
			m_WaitingTurns = 0;
			return;
		}
	}
	else
		// TODO: this could probably be moved before the if entirely, check rounding.
		SetActualSpeed(cmpPosition->GetDistanceTravelled() / dt);

	// we've had to stop at the end of the turn.
	StopMoving();

	// Oops, we've had a problem. Either we were obstructed, or we ran out of path (but still have a goal).
	// Handle it.
	// Failure to handle it will result in stuckness and players complaining.

	if (m_ExpectedPathTicket != 0)
		// wait until we get our path to see where that leads us.
		return;

	// if our next waypoint is close enough to our goal and our goal isn't a point, drop our path and recompute directly.
	if (m_CurrentGoal.IsNotAPoint() && !m_Path.m_Waypoints.empty())
	{
		CmpPtr<ICmpObstructionManager> cmpObstructionManager(GetSystemEntity());
		if (cmpObstructionManager)
		{
			bool inRange = false;
			if (m_CurrentGoal.TargetIsEntity())
				inRange = cmpObstructionManager->IsInTargetRange(m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z,
																 m_CurrentGoal.GetEntity(), m_CurrentGoal.MinRange(), m_CurrentGoal.MaxRange());
			else
				inRange = cmpObstructionManager->IsInPointRange(m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z,
																m_CurrentGoal.X(), m_CurrentGoal.Z(), m_CurrentGoal.MinRange(), m_CurrentGoal.MaxRange());
			if (inRange)
			{
				m_Path.m_Waypoints.clear();
				m_WaitingTurns = MAX_PATH_REATTEMPS; // short path
			}
		}
	}

	// give us some turns to recover.
	// TODO: only do this if we ran into a moving unit and not something else, because something else won't move
	// specifically: if we ran into a moving unit, we should wait a turn and see what happens
	// if we ran into a static unit, recompute a short-path directly
	// if we ran into a static obstruction, recompute long-path directly
	// And then maybe we could add some finetuning based on target.
	if (m_WaitingTurns == 0)
	{
		if (HasValidPath())
			m_WaitingTurns = MAX_PATH_REATTEMPS;
		else
			m_WaitingTurns = 3;
	}

	--m_WaitingTurns;

	// Try again next turn, no changes
	if (m_WaitingTurns >= MAX_PATH_REATTEMPS)
		return;

	// already waited one turn, no changes, so try computing a short path.
	if (m_WaitingTurns >= 3)
	{
		PathGoal goal;
		if (m_Path.m_Waypoints.empty())
			goal = { PathGoal::POINT, m_CurrentGoal.X(), m_CurrentGoal.Z() };
		else
		{
			goal = { PathGoal::POINT, m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z };
			m_Path.m_Waypoints.pop_back();
		}
		RequestShortPath(pos, goal, true);
		return;
	}

	// Last resort, compute a long path
	if (m_WaitingTurns == 2)
	{
		PathGoal goal;
		if (m_Path.m_Waypoints.empty())
			goal = { PathGoal::POINT, m_CurrentGoal.X(), m_CurrentGoal.Z() };
		else
		{
			goal = { PathGoal::POINT, m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z };
			m_Path.m_Waypoints.pop_back();
		}
		RequestLongPath(pos, goal);
		return;
	}


	// m_waitingTurns == 1 here

	// we tried getting a renewed path and still got stuck
	if (m_AbortIfStuck == 0)
	{
		DiscardMove();
		MoveWillFail();
		return;
	}

	--m_AbortIfStuck;

	// Recompute a new path, but wait a few turns first
	m_WaitingTurns = 4 + MAX_PATH_REATTEMPS;

	return;
}

// TODO: ought to be cleverer here.
// In particular maybe we should support some "margin" for error.
bool CCmpUnitMotion::ShouldConsiderOurselvesAtDestination()
{
	if (m_CurrentGoal.TargetIsEntity())
		return IsInTargetRange(m_CurrentGoal.GetEntity(), m_CurrentGoal.MinRange(), m_CurrentGoal.MaxRange());
	else
		return IsInPointRange(m_CurrentGoal.X(),m_CurrentGoal.Z(), m_CurrentGoal.MinRange(), m_CurrentGoal.MaxRange());
}

bool CCmpUnitMotion::PathIsShort(const WaypointPath& path, const CFixedVector2D& from, entity_pos_t minDistance) const
{
	CFixedVector2D prev = from;
	entity_pos_t distLeft = minDistance;

	for (ssize_t i = (ssize_t)path.m_Waypoints.size()-1; i >= 0; --i)
	{
		// Check if the next path segment is longer than the requested minimum
		CFixedVector2D waypoint(path.m_Waypoints[i].x, path.m_Waypoints[i].z);
		CFixedVector2D delta = waypoint - prev;
		if (delta.CompareLength(distLeft) > 0)
			return false;

		// Still short enough - prepare to check the next segment
		distLeft -= delta.Length();
		prev = waypoint;
	}

	// Reached the end of the path before exceeding minDistance
	return true;
}

void CCmpUnitMotion::FaceTowardsPoint(entity_pos_t x, entity_pos_t z)
{
	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (!cmpPosition || !cmpPosition->IsInWorld())
		return;

	CFixedVector2D pos = cmpPosition->GetPosition2D();
	FaceTowardsPointFromPos(pos, x, z);
}

void CCmpUnitMotion::FaceTowardsPointFromPos(const CFixedVector2D& pos, entity_pos_t x, entity_pos_t z)
{
	CFixedVector2D target(x, z);
	CFixedVector2D offset = target - pos;
	if (!offset.IsZero())
	{
		entity_angle_t angle = atan2_approx(offset.X, offset.Y);

		CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
		if (!cmpPosition)
			return;
		cmpPosition->TurnTo(angle);
	}
}

ControlGroupMovementObstructionFilter CCmpUnitMotion::GetObstructionFilter(bool noTarget) const
{
	entity_id_t group = noTarget ? m_Destination.GetEntity() : GetGroup();
	// TODO: if we sometimes want to consider moving units, change here.
	return ControlGroupMovementObstructionFilter(false, group);
}

// TODO: this should be improved, it's a little limited
// EG use of hierarchical pathfinder,…
// also it should probably make the goal passable directly, to avoid conflict with the paths returned.
void CCmpUnitMotion::RequestNewPath()
{
	ENSURE(m_ExpectedPathTicket == 0);

	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (!cmpPosition)
		return;

	// dump current path
	m_Path.m_Waypoints.clear();

	CFixedVector2D position = cmpPosition->GetPosition2D();

#if DISABLE_PATHFINDER
	{
		CmpPtr<ICmpPathfinder> cmpPathfinder (GetSimContext(), SYSTEM_ENTITY);
		CFixedVector2D goalPos = m_Destination.Goal().NearestPointOnGoal(position);
		m_LongPath.m_Waypoints.clear();
		m_ShortPath.m_Waypoints.clear();
		m_ShortPath.m_Waypoints.emplace_back(Waypoint{ goalPos.X, goalPos.Y });
		return;
	}
#endif

	// If it's close then just do a short path, not a long path
	// TODO: If it's close on the opposite side of a river then we really
	// need a long path, so we shouldn't simply check linear distance
	// the check is arbitrary but should be a reasonably small distance.
	// Maybe use PathIsShort?

	// TODO: note by wraitii: figure out if the above comment is still true. It seems false.
	if (m_CurrentGoal.Goal().DistanceToPoint(position) < LONG_PATH_MIN_DIST)
		RequestShortPath(position, m_CurrentGoal.Goal(), true);
	else
		RequestLongPath(position, m_CurrentGoal.Goal());
}

void CCmpUnitMotion::RequestLongPath(const CFixedVector2D& from, const PathGoal& goal)
{
	CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
	if (!cmpPathfinder)
		return;

	// this is by how much our waypoints will be apart at most.
	// this value here seems sensible enough.
	PathGoal improvedGoal = goal;
	improvedGoal.maxdist = SHORT_PATH_MIN_SEARCH_RANGE - entity_pos_t::FromInt(1);

	cmpPathfinder->SetDebugPath(from.X, from.Y, improvedGoal, m_PassClass);

	m_ExpectedPathTicket = cmpPathfinder->ComputePathAsync(from.X, from.Y, improvedGoal, m_PassClass, GetEntityId());
}

void CCmpUnitMotion::RequestShortPath(const CFixedVector2D &from, const PathGoal& goal, bool avoidMovingUnits)
{
	CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
	if (!cmpPathfinder)
		return;

	// wrapping around on m_Tries isn't really a problem so don't check for overflow.
	fixed searchRange = std::max(SHORT_PATH_MIN_SEARCH_RANGE * (++m_Tries + 1), goal.DistanceToPoint(from));
	if (goal.type != PathGoal::POINT && searchRange < goal.hw && searchRange < SHORT_PATH_MIN_SEARCH_RANGE * 2)
		searchRange = std::min(goal.hw, SHORT_PATH_MIN_SEARCH_RANGE * 2);
	if (searchRange > SHORT_PATH_MAX_SEARCH_RANGE)
		searchRange = SHORT_PATH_MAX_SEARCH_RANGE;

	m_ExpectedPathTicket = cmpPathfinder->ComputeShortPathAsync(from.X, from.Y, m_Clearance, searchRange, goal, m_PassClass, avoidMovingUnits, GetGroup(), GetEntityId());
}

bool CCmpUnitMotion::MoveToPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange)
{
	return MoveToPointRange(x, z, minRange, maxRange, INVALID_ENTITY);
}

bool CCmpUnitMotion::MoveToPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange, entity_id_t target)
{
	// Must closely mirror CmpObstructionManager::IsInPointRange
	PROFILE("MoveToPointRange");

	DiscardMove();

	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (!cmpPosition || !cmpPosition->IsInWorld())
		return false;

	CFixedVector2D pos = cmpPosition->GetPosition2D();

	PathGoal goal;
	goal.x = x;
	goal.z = z;

	if (minRange.IsZero() && maxRange.IsZero())
	{
		// Non-ranged movement:

		// Head directly for the goal
		goal.type = PathGoal::POINT;
	}
	else
	{
		// Ranged movement:

		entity_pos_t distance = (pos - CFixedVector2D(x, z)).Length();

		if (distance < minRange)
		{
			// Too close to target - move outwards to a circle
			// that's slightly larger than the min range
			goal.type = PathGoal::INVERTED_CIRCLE;
			goal.hw = minRange + Pathfinding::GOAL_DELTA;
		}
		else if (maxRange >= entity_pos_t::Zero() && distance > maxRange)
		{
			// Too far from target - move inwards to a circle
			// that's slightly smaller than the max range
			goal.type = PathGoal::CIRCLE;
			goal.hw = maxRange - Pathfinding::GOAL_DELTA;

			// If maxRange was abnormally small,
			// collapse the circle into a point
			if (goal.hw <= entity_pos_t::Zero())
				goal.type = PathGoal::POINT;
		}
		else
		{
			// We're already in range - no need to move anywhere
			if (m_FacePointAfterMove)
				FaceTowardsPointFromPos(pos, x, z);
			return false;
		}
	}

	if (target == INVALID_ENTITY)
		m_Destination = SMotionGoal(goal, minRange, maxRange);
	else
		m_Destination = SMotionGoal(GetSimContext(), target, goal, minRange, maxRange);

	m_CurrentGoal = m_Destination;

	RequestNewPath();

	return true;
}

bool CCmpUnitMotion::MoveToTargetRange(entity_id_t target, entity_pos_t minRange, entity_pos_t maxRange)
{
	// Must closely mirror CmpObstructionManager::IsInTargetRange
	PROFILE("MoveToTargetRange");

	DiscardMove();

	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (!cmpPosition || !cmpPosition->IsInWorld())
		return false;

	CFixedVector2D pos = cmpPosition->GetPosition2D();

	CmpPtr<ICmpObstructionManager> cmpObstructionManager(GetSystemEntity());
	if (!cmpObstructionManager)
		return false;

	bool hasObstruction = false;
	ICmpObstructionManager::ObstructionSquare obstruction;
	CmpPtr<ICmpObstruction> cmpObstruction(GetSimContext(), target);
	if (cmpObstruction)
		hasObstruction = cmpObstruction->GetObstructionSquare(obstruction);

	if (!hasObstruction)
	{
		// The target didn't have an obstruction or obstruction shape, so treat it as a point instead

		CmpPtr<ICmpPosition> cmpTargetPosition(GetSimContext(), target);
		if (!cmpTargetPosition || !cmpTargetPosition->IsInWorld())
			return false;

		CFixedVector2D targetPos = cmpTargetPosition->GetPosition2D();

		return MoveToPointRange(targetPos.X, targetPos.Y, minRange, maxRange);
	}

	/*
	* If we're starting outside the maxRange, we need to move closer in.
	* If we're starting inside the minRange, we need to move further out.
	* These ranges are measured from the center of this entity to the edge of the target;
	* we add the goal range onto the size of the target shape to get the goal shape.
	* (Then we extend it outwards/inwards by a little bit to be sure we'll end up
	* within the right range, in case of minor numerical inaccuracies.)
	*
	* There's a bit of a problem with large square targets:
	* the pathfinder only lets us move to goals that are squares, but the points an equal
	* distance from the target make a rounded square shape instead.
	*
	* When moving closer, we could shrink the goal radius to 1/sqrt(2) so the goal shape fits entirely
	* within the desired rounded square, but that gives an unfair advantage to attackers who approach
	* the target diagonally.
	*
	* If the target is small relative to the range (e.g. archers attacking anything),
	* then we cheat and pretend the target is actually a circle.
	* (TODO: that probably looks rubbish for things like walls?)
	*
	* If the target is large relative to the range (e.g. melee units attacking buildings),
	* then we multiply maxRange by approx 1/sqrt(2) to guarantee they'll always aim close enough.
	* (Those units should set minRange to 0 so they'll never be considered *too* close.)
	*/

	CFixedVector2D halfSize(obstruction.hw, obstruction.hh);
	PathGoal goal;
	goal.x = obstruction.x;
	goal.z = obstruction.z;

	entity_pos_t distance = Geometry::DistanceToSquare(pos - CFixedVector2D(obstruction.x, obstruction.z), obstruction.u, obstruction.v, halfSize, true);

	// Compare with previous obstruction
	ICmpObstructionManager::ObstructionSquare previousObstruction;
	cmpObstruction->GetPreviousObstructionSquare(previousObstruction);
	entity_pos_t previousDistance = Geometry::DistanceToSquare(pos - CFixedVector2D(previousObstruction.x, previousObstruction.z), obstruction.u, obstruction.v, halfSize, true);

	bool inside = distance.IsZero() && !Geometry::DistanceToSquare(pos - CFixedVector2D(obstruction.x, obstruction.z), obstruction.u, obstruction.v, halfSize).IsZero();
	if ((distance < minRange && previousDistance < minRange) || inside)
	{
		// Too close to the square - need to move away

		// Circumscribe the square
		entity_pos_t circleRadius = halfSize.Length();

		entity_pos_t goalDistance = minRange + Pathfinding::GOAL_DELTA;

		if (Geometry::ShouldTreatTargetAsCircle(minRange, circleRadius))
		{
			// The target is small relative to our range, so pretend it's a circle
			goal.type = PathGoal::INVERTED_CIRCLE;
			goal.hw = circleRadius + goalDistance;
		}
		else
		{
			goal.type = PathGoal::INVERTED_SQUARE;
			goal.u = obstruction.u;
			goal.v = obstruction.v;
			goal.hw = obstruction.hw + goalDistance;
			goal.hh = obstruction.hh + goalDistance;
		}
	}
	else if (maxRange < entity_pos_t::Zero() || distance < maxRange || previousDistance < maxRange)
	{
		// We're already in range - no need to move anywhere
		FaceTowardsPointFromPos(pos, goal.x, goal.z);
		return false;
	}
	else
	{
		// We might need to move closer:

		// Circumscribe the square
		entity_pos_t circleRadius = halfSize.Length();

		if (Geometry::ShouldTreatTargetAsCircle(maxRange, circleRadius))
		{
			// The target is small relative to our range, so pretend it's a circle

			// Note that the distance to the circle will always be less than
			// the distance to the square, so the previous "distance < maxRange"
			// check is still valid (though not sufficient)
			entity_pos_t circleDistance = (pos - CFixedVector2D(obstruction.x, obstruction.z)).Length() - circleRadius;
			entity_pos_t previousCircleDistance = (pos - CFixedVector2D(previousObstruction.x, previousObstruction.z)).Length() - circleRadius;

			if (circleDistance < maxRange || previousCircleDistance < maxRange)
			{
				// We're already in range - no need to move anywhere
				if (m_FacePointAfterMove)
					FaceTowardsPointFromPos(pos, goal.x, goal.z);
				return false;
			}

			entity_pos_t goalDistance = maxRange - Pathfinding::GOAL_DELTA;

			goal.type = PathGoal::CIRCLE;
			goal.hw = circleRadius + goalDistance;
		}
		else
		{
			// The target is large relative to our range, so treat it as a square and
			// get close enough that the diagonals come within range

			entity_pos_t goalDistance = (maxRange - Pathfinding::GOAL_DELTA)*2 / 3; // multiply by slightly less than 1/sqrt(2)

			goal.type = PathGoal::SQUARE;
			goal.u = obstruction.u;
			goal.v = obstruction.v;
			entity_pos_t delta = std::max(goalDistance, m_Clearance + entity_pos_t::FromInt(TERRAIN_TILE_SIZE)/16); // ensure it's far enough to not intersect the building itself
			goal.hw = obstruction.hw + delta;
			goal.hh = obstruction.hh + delta;
		}
	}

	if (target == INVALID_ENTITY)
		m_Destination = SMotionGoal(goal, minRange, maxRange);
	else
		m_Destination = SMotionGoal(GetSimContext(), target, goal, minRange, maxRange);

	m_CurrentGoal = m_Destination;

	RequestNewPath();

	return true;
}

bool CCmpUnitMotion::IsInPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange)
{
	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (!cmpPosition)
		return false;

	CmpPtr<ICmpObstructionManager> cmpObstructionManager(GetSystemEntity());
	if (!cmpObstructionManager)
		return true; // what's a sane default here?

	CFixedVector2D pos = cmpPosition->GetPosition2D();

	return cmpObstructionManager->IsInPointRange(pos.X, pos.Y, x, z, minRange, maxRange + m_Clearance.Multiply(fixed::FromInt(3)/2));
}

bool CCmpUnitMotion::IsInTargetRange(entity_id_t target, entity_pos_t minRange, entity_pos_t maxRange)
{
	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (!cmpPosition)
		return false;

	CmpPtr<ICmpObstructionManager> cmpObstructionManager(GetSystemEntity());
	if (!cmpObstructionManager)
		return true; // what's a sane default here?

	CFixedVector2D pos = cmpPosition->GetPosition2D();

	return cmpObstructionManager->IsInTargetRange(pos.X, pos.Y, target, minRange, maxRange + m_Clearance.Multiply(fixed::FromInt(3)/2));
}


void CCmpUnitMotion::RenderPath(const WaypointPath& path, std::vector<SOverlayLine>& lines, CColor color)
{
	bool floating = false;
	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (cmpPosition)
		floating = cmpPosition->IsFloating();

	lines.clear();
	std::vector<float> waypointCoords;
	for (size_t i = 0; i < path.m_Waypoints.size(); ++i)
	{
		float x = path.m_Waypoints[i].x.ToFloat();
		float z = path.m_Waypoints[i].z.ToFloat();
		waypointCoords.push_back(x);
		waypointCoords.push_back(z);
		lines.push_back(SOverlayLine());
		lines.back().m_Color = color;
		SimRender::ConstructSquareOnGround(GetSimContext(), x, z, 1.0f, 1.0f, 0.0f, lines.back(), floating);
	}
	float x = cmpPosition->GetPosition2D().X.ToFloat();
	float z = cmpPosition->GetPosition2D().Y.ToFloat();
	waypointCoords.push_back(x);
	waypointCoords.push_back(z);
	lines.push_back(SOverlayLine());
	lines.back().m_Color = color;
	SimRender::ConstructLineOnGround(GetSimContext(), waypointCoords, lines.back(), floating);

}

void CCmpUnitMotion::RenderSubmit(SceneCollector& collector)
{
	if (!m_DebugOverlayEnabled)
		return;

	RenderPath(m_Path, m_DebugOverlayLongPathLines, OVERLAY_COLOR_LONG_PATH);

	for (size_t i = 0; i < m_DebugOverlayLongPathLines.size(); ++i)
		collector.Submit(&m_DebugOverlayLongPathLines[i]);

	for (size_t i = 0; i < m_DebugOverlayShortPathLines.size(); ++i)
		collector.Submit(&m_DebugOverlayShortPathLines[i]);
}
