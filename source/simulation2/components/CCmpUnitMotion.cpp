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

/**
 * Min/Max range to restrict short path queries to. (Larger ranges are slower,
 * smaller ranges might miss some legitimate routes around large obstacles.)
 */
static const entity_pos_t SHORT_PATH_MIN_SEARCH_RANGE = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*2);
static const entity_pos_t SHORT_PATH_MAX_SEARCH_RANGE = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*9);

/**
 * Below this distance to the goal, if we're getting obstructed, we will recreate a brand new Goal for our short-pathfinder
 * Instead of using the one given to us by RecomputeGoalPosition.
 * This is unsafe from a shot/long pathfinder compatibility POV, so it should not be too big.
 */
static const entity_pos_t SHORT_PATH_GOAL_REDUX_DIST = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*2);

/**
 * Minimum distance to goal for a long path request
 * Disabled, see note in RequestNewPath.
 */
static const entity_pos_t LONG_PATH_MIN_DIST = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*0);

/**
 * If we are this close to our target entity/point, then think about heading
 * for it in a straight line instead of pathfinding.
 * TODO: this should probably be reintroduced
 */
static const entity_pos_t DIRECT_PATH_RANGE = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*4);

/**
 * See unitmotion logic for details. Higher means units will retry more often before potentially failing.
 */
static const size_t MAX_PATH_REATTEMPS = 8;

static const CColor OVERLAY_COLOR_PATH(1, 0, 0, 1);

class CCmpUnitMotion : public ICmpUnitMotion
{
private:
	struct SMotionGoal
	{
	private:
		bool m_Valid = false;

		entity_pos_t m_Range;
		entity_id_t m_Entity;
		CFixedVector2D m_Position;

	public:
		SMotionGoal() : m_Valid(false) {};

		SMotionGoal(CFixedVector2D position, entity_pos_t range)
		{
			m_Entity = INVALID_ENTITY;
			m_Range = range;
			m_Position = position;

			m_Valid = true;
		}

		SMotionGoal(entity_id_t target, entity_pos_t range)
		{
			m_Entity = target;
			m_Range = range;

			m_Valid = true;
		}

		template<typename S>
		void SerializeCommon(S& serialize)
		{
			serialize.Bool("valid", m_Valid);

			serialize.NumberFixed_Unbounded("range", m_Range);

			serialize.NumberU32_Unbounded("entity", m_Entity);

			serialize.NumberFixed_Unbounded("x", m_Position.X);
			serialize.NumberFixed_Unbounded("y", m_Position.Y);
		}

		bool IsEntity() const { return m_Entity != INVALID_ENTITY; }
		entity_id_t GetEntity() const { return m_Entity; }

		bool Valid() const { return m_Valid; }
		void Clear() { m_Valid = false; }

		entity_pos_t Range() const { return m_Range; };

		CFixedVector2D GetPosition() const { ENSURE(!m_Entity); return m_Position; }
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
	std::vector<SOverlayLine> m_DebugOverlayPathLines;

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

	// Pathfinder-compliant goal. Should be reachable (at least when it's recomputed).
	PathGoal m_Goal;

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
	bool m_DumpPathOnResult;
	bool m_RunShortPathValidation;

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
		m_DumpPathOnResult = false;
		m_RunShortPathValidation = false;

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
		serialize.Bool("dump path on result", m_DumpPathOnResult);
		serialize.Bool("short path validation", m_RunShortPathValidation);

		SerializeVector<SerializeWaypoint>()(serialize, "path", m_Path.m_Waypoints);

		serialize.NumberU8("tries", m_Tries, 0, 255);
		serialize.NumberU8("waiting turns", m_WaitingTurns, 0, 255);

		serialize.Bool("started moving", m_StartedMoving);

		// strictly speaking this doesn't need to be serialized since it's graphics-only, but it's nicer to.
		serialize.NumberFixed_Unbounded("actual speed", m_ActualSpeed);

		m_Destination.SerializeCommon(serialize);
		m_CurrentGoal.SerializeCommon(serialize);
		SerializeGoal()(serialize, "goal", m_Goal);
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

	// TODO: would be nice to listen to entity renamed messages, but those have no C++ interface so far.
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

	virtual bool SetNewDestinationAsPosition(entity_pos_t x, entity_pos_t z, entity_pos_t range, bool evenUnreachable);
	virtual bool SetNewDestinationAsEntity(entity_id_t target, entity_pos_t range, bool evenUnreachable);

	virtual bool TemporaryRerouteToPosition(entity_pos_t x, entity_pos_t z, entity_pos_t range);
	virtual bool GoBackToOriginalDestination();

	// transform a motion goal into a corresponding PathGoal
	// called by RecomputeGoalPosition
	PathGoal CreatePathGoalFromMotionGoal(const SMotionGoal& motionGoal);

	// take an arbitrary path goal and convert it to a 2D point goal, assign it to m_Goal.
	bool RecomputeGoalPosition(PathGoal& goal);

	virtual void FaceTowardsPoint(entity_pos_t x, entity_pos_t z);
	virtual void FaceTowardsEntity(entity_id_t ent);

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

	void ResetPathfinding()
	{
		m_Tries = 0;
		m_WaitingTurns = 0;

		m_ExpectedPathTicket = 0;
		m_Path.m_Waypoints.clear();
	}

	virtual void DiscardMove()
	{
		StopMoving();

		ResetPathfinding();

		m_CurrentGoal.Clear();
		m_Destination.Clear();
	}

	void MoveWillFail()
	{
		CMessageMoveFailure msg;
		GetSimContext().GetComponentManager().PostMessage(GetEntityId(), msg);
	}

	void MoveHasPaused()
	{
		CMessageMovePaused msg;
		GetSimContext().GetComponentManager().PostMessage(GetEntityId(), msg);
	}

	virtual bool HasValidPath()
	{
		return !m_Path.m_Waypoints.empty();
	}

private:
	CFixedVector2D GetGoalPosition(const SMotionGoal& goal) const
	{
		ENSURE (goal.Valid());

		if (goal.IsEntity())
		{
			CmpPtr<ICmpPosition> cmpPosition(GetSimContext(), goal.GetEntity());
			ENSURE(cmpPosition);
			ENSURE(cmpPosition->IsInWorld()); // TODO: do something? Like return garrisoned building or such?
			return cmpPosition->GetPosition2D();
		}
		else
			return goal.GetPosition();
	}

	bool CurrentGoalHasValidPosition()
	{
		if (!m_CurrentGoal.Valid())
			return false;

		if (m_CurrentGoal.IsEntity())
		{
			CmpPtr<ICmpPosition> cmpPosition(GetSimContext(), m_CurrentGoal.GetEntity());
			if (!cmpPosition || !cmpPosition->IsInWorld())
				return false;
			return true;
		}
		else
			return true;
	}
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

	/**
	 * Handle the result of an asynchronous path query.
	 */
	void PathResult(u32 ticket, const WaypointPath& path);

	/**
	 * Do the per-turn movement and other updates.
	 */
	void Move(fixed dt);

	/**
	 * Check that our current waypoints are sensible or whether we should recompute
	 *
	 * Quick note on how clever UnitMotion should be: UnitMotion should try to reach the current target (m_CurrentGoal)
	 * as well as it can. But it should not take any particular guess on wether something CAN or SHOULD be reached.
	 * Examples: chasing a unit that's faster than us is probably stupid. This is not UnitMotion's to say, UnitMotion should try.
	 * Likewise when requesting a new path, even if it's unreachable unitMotion must try its best (but can inform unitAI that it's being obtuse)
	 * However, if a chased unit is moving, we should try to anticipate its moves by any means possible.
	 */
	void ValidateCurrentPath();

	/**
	 * take a 2D position and return an updated one based on estimated target velocity.
	 */
	inline void UpdatePositionForTargetVelocity(entity_id_t ent, entity_pos_t& x, entity_pos_t& z, fixed& certainty);

	/**
	 * Returns whether we are close enough to the target to assume it's a good enough
	 * position to stop.
	 */
	bool ShouldConsiderOurselvesAtDestination(SMotionGoal& goal);

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
	 */
	ControlGroupMovementObstructionFilter GetObstructionFilter() const;

	/**
	 * Dumps current path and request a new one asynchronously.
	 * Inside of UnitMotion, do not set evenUnreachable to false unless you REALLY know what you're doing.
	 */
	bool RequestNewPath(bool evenUnreachable = true);

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

	if (m_DumpPathOnResult)
	{
		m_Path.m_Waypoints.clear();
		m_DumpPathOnResult = false;
	}

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

	// if this is a short path, verify some things
	// Namely reject any path that takes us in another global region
	// and any waypoint that's not passable/next to a passable cell.
	// this will ensure minimal changes of long/short rance pathfinder discrepancies
	if (m_RunShortPathValidation)
	{
		CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
		ENSURE (cmpPathfinder);

		CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
		ENSURE(cmpPosition);

		u16 i0, j0;
		cmpPathfinder->FindNearestPassableNavcell(cmpPosition->GetPosition2D().X, cmpPosition->GetPosition2D().Y, i0, j0, m_PassClass);

		m_RunShortPathValidation = false;
		for (const Waypoint& wpt : path.m_Waypoints)
		{
			u16 i1, j1;
			u32 dist = cmpPathfinder->FindNearestPassableNavcell(wpt.x, wpt.z, i1, j1, m_PassClass);
			if (dist > 1 || !cmpPathfinder->NavcellIsReachable(i0, j0, i1, j1, m_PassClass))
			{
				MoveWillFail();
				// we will then deal with this on the next Move() call.
				return;
			}
		}
	}

	// if we're currently moving, we have a path, so check if the first waypoint can be removed
	// it's not impossible that we've actually reached it already.
	if (IsActuallyMoving() && path.m_Waypoints.size() >= 2)
	{
		CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
		CFixedVector2D nextWp = CFixedVector2D(path.m_Waypoints.back().x,path.m_Waypoints.back().z) - cmpPosition->GetPosition2D();
		if (nextWp.CompareLength(GetSpeed()/2) <= 0)
		{
			m_Path.m_Waypoints.insert(m_Path.m_Waypoints.end(), path.m_Waypoints.begin(), path.m_Waypoints.end()-1);
			return;
		}
	}
	m_Path.m_Waypoints.insert(m_Path.m_Waypoints.end(), path.m_Waypoints.begin(), path.m_Waypoints.end());
}

void CCmpUnitMotion::ValidateCurrentPath()
{
	// this should be kept in sync with RequestNewPath otherwise we'll spend our whole life repathing.

	// don't validate points, they never change
	if (!m_CurrentGoal.IsEntity())
		return;

	// TODO: figure out what to do when the goal dies.
	// for now we'll keep on keeping on, but reset as if our goal was a position
	// and send a failure message to UnitAI in case it wants to do something
	// use position as a proxy for existence
	CmpPtr<ICmpPosition> cmpTargetPosition(GetSimContext(), m_CurrentGoal.GetEntity());
	if (!cmpTargetPosition || !cmpTargetPosition->IsInWorld())
	{
		// TODO: this should call a custom function for this
		SMotionGoal newGoal(CFixedVector2D(m_Goal.x, m_Goal.z), m_CurrentGoal.Range());
		m_Destination = newGoal;
		m_CurrentGoal = newGoal;
		RequestNewPath();
		MoveWillFail();
		return;
	}

	// don't validate if no path.
	if (!HasValidPath())
		return;

	// TODO: check LOS here (instead of in UnitAI like we do now).

	// if our goal can move, then perhaps it has.
	CmpPtr<ICmpUnitMotion> cmpTargetUnitMotion(GetSimContext(), m_CurrentGoal.GetEntity());
	if (!cmpTargetUnitMotion)
		return;

	// Check if our current Goal's position (ie m_Goal, not m_CurrentGoal) is sensible.

	// TODO: this will probably be called every turn if the entity tries to go to an unreachable unit
	// In those cases, UnitAI should be warned that the unit is unreachable and tell us to do something else.

	CFixedVector2D targetPos = cmpTargetPosition->GetPosition2D();
	fixed certainty = m_Clearance*2;
	UpdatePositionForTargetVelocity(m_CurrentGoal.GetEntity(), targetPos.X, targetPos.Y, certainty);

	CmpPtr<ICmpObstructionManager> cmpObstructionManager(GetSystemEntity());
	if (!cmpObstructionManager->IsPointInPointRange(m_Goal.x, m_Goal.z, targetPos.X, targetPos.Y, m_CurrentGoal.Range() - certainty, m_CurrentGoal.Range() + certainty))
		RequestNewPath();
}

void CCmpUnitMotion::UpdatePositionForTargetVelocity(entity_id_t ent, entity_pos_t& x, entity_pos_t& z, fixed& certainty)
{
	CmpPtr<ICmpPosition> cmpTargetPosition(GetSimContext(), ent);
	CmpPtr<ICmpUnitMotion> cmpTargetUnitMotion(GetSimContext(), ent);
	if (!cmpTargetPosition || !cmpTargetPosition->IsInWorld() || !cmpTargetUnitMotion || !cmpTargetUnitMotion->IsActuallyMoving())
		return;

	// So here we'll try to estimate where the unit will be by the time we reach it.
	// This can be done perfectly but I cannot think of a non-iterative process and this seems complicated for our purposes here
	// so just get our direct distance and do some clever things, we'll correct later on anyhow so it doesn't matter.
	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());

	// try to estimate in how much time we'll reach it.
	fixed distance = (cmpTargetPosition->GetPosition2D() - cmpPosition->GetPosition2D()).Length();
	fixed time = std::min(distance / GetSpeed(), fixed::FromInt(5)); // don't try from too far away or this is just dumb.

	CFixedVector2D travelVector = (cmpTargetPosition->GetPosition2D() - cmpTargetPosition->GetPreviousPosition2D()).Multiply(time) * 2;
	x += travelVector.X;
	z += travelVector.Y;

	certainty += time * 2;
}

// TODO: this can probably be split in a few functions efficiently and it'd be cleaner.
void CCmpUnitMotion::Move(fixed dt)
{
	PROFILE("Move");

	// early out
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

	// All path updates/checks should go here, before the moving loop.
	ValidateCurrentPath();

	//////////////////////////////////////////////////////////////////////////////////////
	//// AFTER THIS POINT, NO MESSAGES SHOULD BE SENT OR HORRIBLE THINGS WILL HAPPEN. ////
	////                             YOU HAVE BEEN WARNED.                            ////
	//////////////////////////////////////////////////////////////////////////////////////

	// validate our state following the messages potentially sent above.
	if (!IsTryingToMove() || !cmpPosition || !cmpPosition->IsInWorld() || !CurrentGoalHasValidPosition())
	{
		// One of the messages we sent UnitAI caused us to stop moving entirely.
		// Tell the visual actor we're not moving this turn to avoid gliding.
		SetActualSpeed(fixed::Zero());
		return;
	}

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

	// Anticipate here future pathing needs;
	bool willBeObstructed = false;
	if (!wasObstructed && HasValidPath() && !cmpPathfinder->CheckMovement(GetObstructionFilter(), pos.X, pos.Y, m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z, m_Clearance, m_PassClass))
		willBeObstructed = true;

	// Update the Position component after our movement (if we actually moved anywhere)
	if (pos != initialPos)
	{
		CFixedVector2D offset = pos - initialPos;

		// tell other components and visual actor we are moving.
		if (!m_StartedMoving)
			StartMoving();

		// Face towards the target
		entity_angle_t angle = atan2_approx(offset.X, offset.Y);
		cmpPosition->MoveAndTurnTo(pos.X,pos.Y, angle);

		// Calculate the mean speed over this past turn.
		// TODO: this is often just a little different from our actual top speed
		// so we end up changing the actual speed quite often, which is a little silly.
		SetActualSpeed(cmpPosition->GetDistanceTravelled() / dt);

		if (!wasObstructed && !willBeObstructed)
		{
			// everything is going smoothly, return.
			m_Tries = 0;
			m_WaitingTurns = 0;
			return;
		}
	}
	else
		// TODO: this and the same call in the if above could probably be moved before the if entirely, check rounding.
		SetActualSpeed(fixed::Zero());

	//////////////////////////////////////////////////////////////////////////
	////    From this point onwards messages are "safe" to send again.    ////
	////  But after any messages are sent, you should validate our state. ////
	//////////////////////////////////////////////////////////////////////////

	// we've had to stop at the end of the turn.
	if (wasObstructed && m_StartedMoving)
	{
		StopMoving();

		MoveHasPaused();
		// Validate that the MoveHasPaused message hint has not caused us to be in an invalid state.
		if (!IsTryingToMove() || !cmpPosition || !cmpPosition->IsInWorld() || !CurrentGoalHasValidPosition())
			return;
	}

	if (ShouldConsiderOurselvesAtDestination(m_CurrentGoal))
		// If we're out of path (ie not moving) but have a valid destination (IsTryingToMove()), we'll end up here every turn.
		// We should not repath if we actually are where we want to be (ie at destination).
		// That we do this means that the range check performed by whoever is calling us must be more permissive than this one though
		// otherwise we'll never reach our target.
		return;

	// Oops, we have a problem. Either we were obstructed, we plan to be obstructed soon, or we ran out of path (but still have a goal).
	// Handle it.
	// Failure to handle it will result in stuckness and players complaining.

	if (m_ExpectedPathTicket != 0)
		// wait until we get our path to see where that leads us.
		return;

	// TODO: it would be nice to react differently to different problems
	// eg running into a static obstruction can be handled by the long-range pathfinder
	// but running in a unit cannot.
	if (m_WaitingTurns == 0)
	{
		if (HasValidPath())
			m_WaitingTurns = MAX_PATH_REATTEMPS; // currently we won't wait at all
		else
			m_WaitingTurns = 3;
	}

	--m_WaitingTurns;

	// MAX_PATH_REATTEMPS and above: we wait.
	if (m_WaitingTurns >= MAX_PATH_REATTEMPS)
		return;

	// If we're here we want to compute a short path.
	if (m_WaitingTurns >= 3)
	{
		if (m_Path.m_Waypoints.empty())
		{
			RequestNewPath();
			return;
		}
		/**
		 * Here there are two cases:
		 * 1) We are somewhat far away from the goal
		 * 2) We're really close to the goal.
		 * If it's (2) it's likely that we are running into units that are currently doing the same thing we want to do (gathering from the same tree…)
		 * Since the initial call to MakeGoalReachable gave us a specific 2D coordinate, and we can't reach it,
		 * We have a relatively high chance of never being able to reach that particular point.
		 * So we need to recreate the actual goal for this entity. This is a little dangerous in terms of short/long pathfinder compatibility
		 * So we'll run sanity checks on the output to try and not get stuck/go where we shouldn't.
		 */
		PathGoal goal;

		CFixedVector2D nextWptPos(m_Path.m_Waypoints.front().x, m_Path.m_Waypoints.front().z);
		bool redraw = true;
		if ((nextWptPos - pos).CompareLength(SHORT_PATH_GOAL_REDUX_DIST) > 0)
		{
			// Check wether our next waypoint is obstructed in which case skip it.
			// TODO: would be good to have a faster function here.
			if (!cmpPathfinder->CheckMovement(GetObstructionFilter(), m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z, m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z, m_Clearance, m_PassClass))
				m_Path.m_Waypoints.pop_back();
			if (!m_Path.m_Waypoints.empty())
			{
				redraw = false;
				goal = { PathGoal::POINT, m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z };
				m_Path.m_Waypoints.pop_back();
			}
		}
		if (redraw)
		{
			goal = CreatePathGoalFromMotionGoal(m_CurrentGoal);
			m_DumpPathOnResult = true;
		}

		RequestShortPath(pos, goal, true);
		return;
	}

	// Last resort, compute a long path
	if (m_WaitingTurns == 2)
	{
		if (m_Path.m_Waypoints.empty())
		{
			RequestNewPath();
			return;
		}
		PathGoal goal;
		goal = { PathGoal::POINT, m_Path.m_Waypoints.back().x, m_Path.m_Waypoints.back().z };
		m_Path.m_Waypoints.pop_back();

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

// Only used to send a "hint" to unitAI.
bool CCmpUnitMotion::ShouldConsiderOurselvesAtDestination(SMotionGoal& goal)
{
	if (HasValidPath())
		return false; // wait until we're done.

	CmpPtr<ICmpObstructionManager> cmpObstructionManager(GetSystemEntity());
	if (!cmpObstructionManager)
		return true; // what's a sane default here?

	if (goal.IsEntity())
		return cmpObstructionManager->IsInTargetRange(GetEntityId(), goal.GetEntity(), goal.Range(), goal.Range());
	else
		return cmpObstructionManager->IsInPointRange(GetEntityId(), goal.GetPosition().X, goal.GetPosition().Y, goal.Range(), goal.Range());
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

void CCmpUnitMotion::FaceTowardsEntity(entity_id_t ent)
{
	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	if (!cmpPosition || !cmpPosition->IsInWorld())
		return;

	CmpPtr<ICmpPosition> cmpTargetPosition(GetSimContext(), ent);
	if (!cmpTargetPosition || !cmpTargetPosition->IsInWorld())
		return;

	CFixedVector2D pos = cmpPosition->GetPosition2D();
	CFixedVector2D targetPos = cmpTargetPosition->GetPosition2D();

	CFixedVector2D offset = targetPos - pos;
	if (!offset.IsZero())
	{
		entity_angle_t angle = atan2_approx(offset.X, offset.Y);
		cmpPosition->TurnTo(angle);
	}

}

ControlGroupMovementObstructionFilter CCmpUnitMotion::GetObstructionFilter() const
{
	// TODO: if we sometimes want to consider moving units, change here.
	return ControlGroupMovementObstructionFilter(true, GetGroup());
}

// TODO: this can be improved, it's a little limited
// e.g. use of hierarchical pathfinder,…
// Also adding back the "straight-line if close enough" test could be good.
bool CCmpUnitMotion::RequestNewPath(bool evenUnreachable)
{
	ENSURE(m_ExpectedPathTicket == 0);

	ENSURE(CurrentGoalHasValidPosition());

	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	ENSURE (cmpPosition);

	CFixedVector2D position = cmpPosition->GetPosition2D();

	m_DumpPathOnResult = true;

	bool reachable = RecomputeGoalPosition(m_Goal);

	if (!reachable && !evenUnreachable)
	{
		// Do not submit a path request if we've been told it's not going to be used anyhow.
		DiscardMove();
		return false;
	}

	ENSURE(m_Goal.x >= fixed::Zero());

	/**
	 * A (long) note on short vs long range pathfinder, their conflict, and stuck units.
	 * A long-standing issue with 0 A.D.'s pathfinding has been that the short-range pathfinder is "better" than the long-range
	 * Indeed it can find paths that the long-range one cannot, since the grid is coarser than the real vector representation.
	 * This leads to units going where they shouldn't go, notably impassable and "unreachable" areas.
	 * This has been a -real- plague. Made worse by the facts that groups of trees tended to trigger it, leading to stuck gatherers…
	 * Thus, in general, we'd want the short-range and the long-range pathfinder to coincide. But making the short-range pathfinder
	 * register all impassable navcells as edges would just be way too slow, so we can't do that, so we -cannot- fix the issue
	 * by just changing the pathfinders' behavior.
	 *
	 * All hope is not lost, however.
	 *
	 * A big part of the problem is that before the unitMotion rewrite, UnitMotion requested a path to the goal, and then the pathfinder
	 * made that goal "reachable" by calling MakeGoalReachable, which uses the same grid as the long-range pathfinder. Thus, over short ranges,
	 * the pathfinder entirely short-circuited this. Since UnitMotion now calls MakeGoalReachable on its own, it only ever requests
	 * paths to points that are indeed supposed to be reachable. This does fix a number of cases.
	 *
	 * But then, why set LONG_PATH_MIN_DIST to 0 and disable the use of short paths here? Well it turns out you still had a few edge cases.
	 *
	 * Imagine two houses next to each other, with a space between them just wide enough that there are no passable navcells,
	 * but enough space for the short-range pathfinder to return a path through them (make them in a test map if you have to).
	 * If you ask a unit to cross there, the goal won't change: it's reachable by the long-range pathfinder by going around the house.
	 * However, the distance is < LONG_PATH_MIN_DIST, so the short-range pathfinder is called, so it goes through the house. Edge case.
	 * There's a variety of similar cases that can be imagined around the idea that there exists a shorter path visible only by the short-range pathfinder.
	 * If we never use the short-pathfinder in RequestNewPath, we can safely avoid those edge cases.
	 *
	 * However, we still call the short-pathfinder when running into an obstruction to avoid units. Can't that get us stuck too?
	 * Well, it probably can. But there's a few things to consider:
	 * -It's harder to trigger it if you actually have to run into a unit
	 * -In those cases, UnitMotion requests a path to the next existing waypoint (if there are none, it calls requestnewPath to get those)
	 * and the next existing waypoint has -necessarily- been given to us by the long-range pathfinder since we're using it here
	 * -We are running sanity checks on the output (see PathResult).
	 * Thus it's far less likely that the short-range pathfinder will return us an impassable path.
	 * It -is- not entirely impossible. A freak construction with many units strategically positionned could probably reveal the bug.
	 * But it's in my opinion rare enough that this discrepancy can be considered fixed.
	 */

	if (m_Goal.DistanceToPoint(position) < LONG_PATH_MIN_DIST)
		RequestShortPath(position, m_Goal, true);
	else
		RequestLongPath(position, m_Goal);

	return reachable;
}

PathGoal CCmpUnitMotion::CreatePathGoalFromMotionGoal(const SMotionGoal& motionGoal)
{
	PathGoal goal = PathGoal();
	goal.x = fixed::FromInt(-1); // to figure out whether it's false-unreachable or false-buggy

	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	ENSURE(cmpPosition);

	CFixedVector2D pos = cmpPosition->GetPosition2D();

	// The point of this function is to get a reachable navcell where we want to go.
	// It calls the hierarchical pathfinder's MakeGoalReachable function directly
	// and analyzes to result to return something acceptable.
	// "acceptable" means that if there is a path, once the unit has reached its destination,
	// the ObstructionManager's "IsInPointRange/IsInTargetRange" should consider it in range.
	// So we need to make sure MakeGoalReachable will return something in sync.

	// defaut to point at position
	goal.type = PathGoal::POINT;
	goal.x = GetGoalPosition(motionGoal).X;
	goal.z = GetGoalPosition(motionGoal).Y;

	// few cases to consider.
	if (motionGoal.IsEntity())
	{
		CmpPtr<ICmpObstruction> cmpObstruction(GetSimContext(), motionGoal.GetEntity());
		if (cmpObstruction)
		{
			ICmpObstructionManager::ObstructionSquare obstruction;
			bool hasObstruction = cmpObstruction->GetObstructionSquare(obstruction);
			if (hasObstruction)
			{
				fixed certainty;
				UpdatePositionForTargetVelocity(motionGoal.GetEntity(), obstruction.x, obstruction.z, certainty);

				goal.type = PathGoal::CIRCLE;
				goal.x = obstruction.x;
				goal.z = obstruction.z;
				goal.hw = obstruction.hw + motionGoal.Range() + m_Clearance;

				// if not a unit, treat as a square
				if (cmpObstruction->GetUnitRadius() == fixed::Zero())
				{
					goal.type = PathGoal::SQUARE;
					goal.hh = obstruction.hh + motionGoal.Range() + m_Clearance;
					goal.u = obstruction.u;
					goal.v = obstruction.v;

					fixed distance = Geometry::DistanceToSquare(pos - CFixedVector2D(goal.x,goal.z), goal.u, goal.v, CFixedVector2D(goal.hw, goal.hh), true);
					if (distance == fixed::Zero())
						goal.type = PathGoal::INVERTED_SQUARE;
				}
				else if ((pos - CFixedVector2D(goal.x,goal.z)).CompareLength(goal.hw) <= 0)
					goal.type = PathGoal::INVERTED_CIRCLE;
			}
		}
		// if no obstruction, keep treating as a point
	}
	if (goal.type == PathGoal::POINT && motionGoal.Range() > fixed::Zero())
	{
		goal.type = PathGoal::CIRCLE;
		goal.hw = motionGoal.Range();
		if ((pos - CFixedVector2D(goal.x,goal.z)).CompareLength(goal.hw) <= 0)
			goal.type = PathGoal::INVERTED_CIRCLE;
	}

	return goal;
}

bool CCmpUnitMotion::RecomputeGoalPosition(PathGoal& goal)
{
	if (!CurrentGoalHasValidPosition())
		return false; // we're not going anywhere

	goal = CreatePathGoalFromMotionGoal(m_CurrentGoal);

	// We now have a correct goal.
	// Make it reachable

	CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
	ENSURE(cmpPathfinder);

	CmpPtr<ICmpPosition> cmpPosition(GetEntityHandle());
	ENSURE(cmpPosition);

	CFixedVector2D pos = cmpPosition->GetPosition2D();

	bool reachable = cmpPathfinder->MakeGoalReachable(pos.X, pos.Y, goal, m_PassClass);

	// TODO: ought to verify that the returned navcell is in range if it's reachable as a sanity check

	m_Goal = goal;

	return reachable;
}

void CCmpUnitMotion::RequestLongPath(const CFixedVector2D& from, const PathGoal& goal)
{
	CmpPtr<ICmpPathfinder> cmpPathfinder(GetSystemEntity());
	if (!cmpPathfinder)
		return;

	m_RunShortPathValidation = false;

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

	m_RunShortPathValidation = true;

	// wrapping around on m_Tries isn't really a problem so don't check for overflow.
	fixed searchRange = std::max(SHORT_PATH_MIN_SEARCH_RANGE * (++m_Tries + 1), goal.DistanceToPoint(from));
	if (goal.type != PathGoal::POINT && searchRange < goal.hw && searchRange < SHORT_PATH_MIN_SEARCH_RANGE * 2)
		searchRange = std::min(goal.hw, SHORT_PATH_MIN_SEARCH_RANGE * 2);
	if (searchRange > SHORT_PATH_MAX_SEARCH_RANGE)
		searchRange = SHORT_PATH_MAX_SEARCH_RANGE;

	m_ExpectedPathTicket = cmpPathfinder->ComputeShortPathAsync(from.X, from.Y, m_Clearance, searchRange, goal, m_PassClass, avoidMovingUnits, GetGroup(), GetEntityId());
}


bool CCmpUnitMotion::SetNewDestinationAsPosition(entity_pos_t x, entity_pos_t z, entity_pos_t range, bool evenUnreachable)
{
	// This sets up a new destination, scrap whatever came before.
	DiscardMove();

	m_Destination = SMotionGoal(CFixedVector2D(x, z), range);
	m_CurrentGoal = m_Destination;

	bool reachable = RequestNewPath(evenUnreachable); // calls RecomputeGoalPosition

	return reachable;
}

bool CCmpUnitMotion::SetNewDestinationAsEntity(entity_id_t ent, entity_pos_t range, bool evenUnreachable)
{
	// This sets up a new destination, scrap whatever came before.
	DiscardMove();

	// validate entity's existence.
	CmpPtr<ICmpPosition> cmpPosition(GetSimContext(), ent);
	if (!cmpPosition || !cmpPosition->IsInWorld())
		return false;

	m_Destination = SMotionGoal(ent, range);
	m_CurrentGoal = m_Destination;

	bool reachable = RequestNewPath(evenUnreachable); // calls RecomputeGoalPosition

	return reachable;
}

bool CCmpUnitMotion::TemporaryRerouteToPosition(entity_pos_t x, entity_pos_t z, entity_pos_t range)
{
	// Does not reset destination, this is a temporary rerouting.
	StopMoving();
	ResetPathfinding();
	m_CurrentGoal = SMotionGoal(CFixedVector2D(x, z), range);

	// we must set evenUnreachable to true otherwise if the path is unreachable we'll DiscardMove() and we don't want to do that.
	bool reachable = RequestNewPath(true); // calls RecomputeGoalPosition

	// if this is false, whoever called this function should probably un-reroute us and let us go on our way.
	return reachable;
}

bool CCmpUnitMotion::GoBackToOriginalDestination()
{
	SMotionGoal original = m_Destination;

	// assume evenUnreachable, since if it was false originally we'd have stopped by now.
	if (original.IsEntity())
		return SetNewDestinationAsEntity(original.GetEntity(), original.Range(), true);
	else
		return SetNewDestinationAsPosition(original.GetPosition().X,original.GetPosition().Y, original.Range(), true);
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

	if (CurrentGoalHasValidPosition())
	{
		float x = GetGoalPosition(m_CurrentGoal).X.ToFloat();
		float z = GetGoalPosition(m_CurrentGoal).Y.ToFloat();
		lines.push_back(SOverlayLine());
		lines.back().m_Color = CColor(0.0f, 1.0f, 0.0f, 1.0f);
		SimRender::ConstructSquareOnGround(GetSimContext(), x, z, 1.0f, 1.0f, 0.4f, lines.back(), floating);
	}

	float x = m_Goal.x.ToFloat();
	float z = m_Goal.z.ToFloat();
	lines.push_back(SOverlayLine());
	lines.back().m_Color = CColor(0.0f, 1.0f, 1.0f, 1.0f);
	SimRender::ConstructSquareOnGround(GetSimContext(), x, z, 1.0f, 1.0f, 0.0f, lines.back(), floating);

	x = cmpPosition->GetPosition2D().X.ToFloat();
	z = cmpPosition->GetPosition2D().Y.ToFloat();
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

	RenderPath(m_Path, m_DebugOverlayPathLines, OVERLAY_COLOR_PATH);

	for (size_t i = 0; i < m_DebugOverlayPathLines.size(); ++i)
		collector.Submit(&m_DebugOverlayPathLines[i]);
}
