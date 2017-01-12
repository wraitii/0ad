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

#ifndef INCLUDED_ICMPUNITMOTION
#define INCLUDED_ICMPUNITMOTION

#include "simulation2/system/Interface.h"

#include "simulation2/components/ICmpPathfinder.h" // for pass_class_t
#include "simulation2/components/ICmpPosition.h" // for entity_pos_t

/**
 * Motion interface for entities with complex movement capabilities.
 * (Simpler motion is handled by ICmpMotion instead.)
 *
 * This component is designed to handle clever individual movement,
 * not movement as part of an integrated group (formation, bataillon…)
 * and another component should probably be designed for that.
 */
class ICmpUnitMotion : public IComponent
{
public:

	/**
	 * Resets motion and assigns a new 2D position as destination.
	 * Returns false if the position is unreachable (or if the move could not be completed for any other reason).
	 * Otherwise, returns true.
	 * If evenUnreachable is false, and the point is unreachable, then the unit will not start moving.
	 * Otherwise, the unit will try to go to another position as close as possible to the destination.
	 */
	virtual bool SetNewDestinationAsPosition(entity_pos_t x, entity_pos_t z, entity_pos_t range, bool evenUnreachable) = 0;

	/**
	 * Resets motion and assigns a new entity as destination.
	 * Returns false if the entity is unreachable (or if the move could not be completed for any other reason).
	 * Otherwise, returns true.
	 * If evenUnreachable is false, and the point is unreachable, then the unit will not start moving.
	 * Otherwise, the unit will try to go to another position as close as possible to the destination.
	 */
	virtual bool SetNewDestinationAsEntity(entity_id_t target, entity_pos_t range, bool evenUnreachable) = 0;

	/**
	 * Set m_CurrentGoal to the given position. This does not affect m_Destination.
	 * This does not reset the current move or send any specific message.
	 */
	virtual bool TemporaryRerouteToPosition(entity_pos_t x, entity_pos_t z, entity_pos_t range) = 0;

	/**
	 * Resets our pathfinding towards the original m_Destination.
	 * If it wasn't modified by a TemporaryRerouteToPosition, this does nothing (except potentially stop the unit for one turn).
	 * returns the same as the original SetNewDestinationAsXYZ call, with evenUnreachable set to true.
	 */
	virtual bool GoBackToOriginalDestination() = 0;

	/**
	 * Turn to look towards the given point.
	 */
	virtual void FaceTowardsPoint(entity_pos_t x, entity_pos_t z) = 0;

	/**
	 * Turn to look towards the given entity.
	 */
	virtual void FaceTowardsEntity(entity_id_t ent) = 0;

	/**
	 * Determine whether to abort or retry X times if pathing fails.
	 * Generally safer to let it abort and inform us.
	 */
	virtual void SetAbortIfStuck(u8 shouldAbort) = 0;

	/**
	 * Stops the unit. Does not clear the destination
	 * so the unit may start moving again next turn.
	 * Mostly used internally but exposed if anybody wants to stop for whatever reason.
	 */
	virtual void StopMoving() = 0;

	/**
	 * Stop moving, clear any destination, path, and ticket pending.
	 * Basically resets the unit's motion.
	 * Won't send any message.
	 */
	virtual void DiscardMove() = 0;

	/**
	 * Asks wether the unit has a path to follow
	 */
	virtual bool HasValidPath() = 0;

	/**
	 * Get how much faster/slower we are at than normal.
	 */
	virtual fixed GetSpeedRatio() = 0;

	/**
	 * Get how much faster than our regular speed we can go.
	 */
	virtual fixed GetTopSpeedRatio() = 0;

	/**
	 * Set the current movement speed.
	 * 'speed' in % of top speed (ie 3.0 will be 3 times top speed).
	 */
	virtual void SetSpeed(fixed speed) = 0;

	/**
	 * Get whether the unit is actually moving on the map this turn.
	 */
	virtual bool IsActuallyMoving() = 0;

	/**
	 * Get whether a unit is trying to go somewhere
	 * NB: this does not mean its position is actually changing right now.
	 */
	virtual bool IsTryingToMove() = 0;

	/**
	 * Get the unit theoretical speed in metres per second.
	 * GetActualSpeed will return historical speed
	 * This is affected by SetSpeed.
	 */
	virtual fixed GetSpeed() = 0;

	/**
	 * Get the unit base/walk speed in metres per second.
	 * This is NOT affected by SetSpeed.
	 */
	virtual fixed GetBaseSpeed() = 0;

	/**
	 * Set whether the unit will turn to face the target point after finishing moving.
	 */
	virtual void SetFacePointAfterMove(bool facePointAfterMove) = 0;

	/**
	 * Get the unit's passability class.
	 */
	virtual pass_class_t GetPassabilityClass() = 0;

	/**
	 * Get the passability class name (as defined in pathfinder.xml)
	 */
	virtual std::string GetPassabilityClassName() = 0;

	/**
	 * Get the unit clearance (used by the Obstruction component)
	 */
	virtual entity_pos_t GetUnitClearance() = 0;

	/**
	 * Toggle the rendering of debug info.
	 */
	virtual void SetDebugOverlay(bool enabled) = 0;

	DECLARE_INTERFACE_TYPE(UnitMotion)
};

#endif // INCLUDED_ICMPUNITMOTION
