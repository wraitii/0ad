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
 * It should eventually support different movement speeds, moving to areas
 * instead of points, moving as part of a group, moving as part of a formation,
 * etc.
 */
class ICmpUnitMotion : public IComponent
{
public:

	/**
	 * Attempt to walk into range of a to a given point, or as close as possible.
	 * The range is measured from the center of the unit.
	 * If the unit is already in range, or cannot move anywhere at all, or if there is
	 * some other error, then returns false.
	 * Otherwise, returns true and sends a MotionChanged message after starting to move,
	 * and sends another MotionChanged after finishing moving.
	 * If maxRange is negative, then the maximum range is treated as infinity.
	 */
	virtual bool MoveToPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange) = 0;

	/**
	 * Determine wether the givven point is within the given range, using the same measurement
	 * as MoveToPointRange.
	 */
	virtual bool IsInPointRange(entity_pos_t x, entity_pos_t z, entity_pos_t minRange, entity_pos_t maxRange) = 0;

	/**
	 * Determine whether the target is within the given range, using the same measurement
	 * as MoveToTargetRange.
	 */
	virtual bool IsInTargetRange(entity_id_t target, entity_pos_t minRange, entity_pos_t maxRange) = 0;

	/**
	 * Attempt to walk into range of a given target entity, or as close as possible.
	 * The range is measured between approximately the edges of the unit and the target, so that
	 * maxRange=0 is not unreachably close to the target.
	 * If the unit is already in range, or cannot move anywhere at all, or if there is
	 * some other error, then returns false.
	 * Otherwise, returns true and sends a MotionChanged message after starting to move,
	 * and sends another MotionChanged after finishing moving.
	 * If maxRange is negative, then the maximum range is treated as infinity.
	 */
	virtual bool MoveToTargetRange(entity_id_t target, entity_pos_t minRange, entity_pos_t maxRange) = 0;

	/**
	 * Turn to look towards the given point.
	 */
	virtual void FaceTowardsPoint(entity_pos_t x, entity_pos_t z) = 0;

	/**
	 * Determine whether to abort or retry infinitely if pathing fails.
	 * Generally safer to let it abort and inform us.
	 */
	virtual void SetAbortIfStuck(bool shouldAbort) = 0;

	/**
	 * Stop moving immediately, don't send messages.
	 * This should be used if you are going to ask for a new path,
	 * in the same function, for example.
	 * In doubt, UnitAI should probably call this.
	 * Use with caution.
	 */
	virtual void DiscardMove() = 0;

	/**
	 * Stop moving immediately, send messages.
	 * In doubt, components that are not UnitIA should probably call this.
	 */
	virtual void CompleteMove() = 0;

	/**
	 * Get the movement speed from last turn to this turn
	 * This is effectively historical data
	 * And not a good indicator of whether we are actually moving,
	 * Prefer IsActuallyMoving
	 */
	virtual fixed GetActualSpeed() = 0;

	/**
	 * Get how much faster/slower we are at than normal.
	 */
	virtual fixed GetSpeedRatio() = 0;

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
	 * Get the unit template speed in metres per second.
	 * This is NOT affected by SetSpeed.
	 */
	virtual fixed GetTemplateSpeed() = 0;

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
