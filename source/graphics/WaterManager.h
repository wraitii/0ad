/* Copyright (C) 2021 Wildfire Games.
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

#ifndef INCLUDED_WATERMANAGER
#define INCLUDED_WATERMANAGER

#include "graphics/Color.h"
#include "graphics/Texture.h"

class WaterRendering;

class WaterManager
{
	friend class WaterRendering;
public:
	// Water parameters
	// TODO: possibly make these private at some point.
	CColor m_WaterColor;	// Color of the water without refractions. This is what you're seeing when the water's deep or murkiness high.
	CColor m_WaterTint;		// Tint of refraction in the water.
	float m_Waviness;		// How big the waves are.
	float m_Murkiness;		// How murky the water is.
	float m_WindAngle;	// In which direction the water waves go.

public:
	WaterManager();
	~WaterManager();

	std::wstring GetWaterType() const { return m_WaterType; };
	void SetWaterType(std::wstring type);

	float GetWaterHeight() const { return m_WaterHeight; }
	void SetWaterHeight(float height);

	bool IsRenderingEnabled() const { return m_RenderWater; }
	void SetRenderingEnabled(bool enabled);

	float GetWindStrength(int x, int z) const;

	/**
	 * Updates the map size. Will trigger a complete recalculation of fancy water information the next turn.
	 */
	void SetMapSize(size_t size);

	/**
	 * Mark an area of the terrain as needing recalculations.
	 */
	void MarkDirty(i32 i0, i32 i1, i32 j0, i32 j1);

	void RecomputeWaterDataIfNeeded() { if (m_NeedRecomputation) RecomputeWaterData(); }
private:
	/**
	 * RecomputeWaterData: calculates all derived data from the waterheight or the map size.
	 */
	void RecomputeWaterData();

	/**
	 * RecomputeWindStrength: calculates the intensity of waves
	 */
	void RecomputeWindStrength();

	/**
	 * RecomputeDistanceHeightmap: recalculates (or calculates) the distance heightmap.
	 */
	void RecomputeDistanceHeightmap();

	// Which texture to use for water rendering.
	std::wstring m_WaterType;

	float* m_WindStrength;	// How strong the waves are at point X. % of waviness.
	float* m_DistanceHeightmap; // How far from the shore a point is. Manhattan
	CVector3D* m_BlurredNormalMap;	// Cache a slightly blurred map of the normals of the terrain.

	size_t m_MapSize;

	// Enable/Disable water rendering altogether.
	bool m_RenderWater;

	// True if RecomputeWaterData() should be called before rendering.
	bool m_NeedRecomputation;

	// Should follow the simulation's water height.
	float m_WaterHeight;
};

#endif // INCLUDED_WATERMANAGER
