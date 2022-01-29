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

#ifndef INCLUDED_WATER
#define INCLUDED_WATER

#include "graphics/Color.h"
#include "graphics/RenderableObject.h"
#include "graphics/Terrain.h"

#include "renderer/WaterRData.h"
#include <memory>

//class WaterRData;

class CWater
{
public:
	CWater(CTerrain& terrain) : m_Terrain(terrain), m_WaterRData(*this)
	{
		m_WindAngle = 0.0f;
		m_Waviness = 8.0f;
		m_WaterColor = CColor(0.3f, 0.35f, 0.7f, 1.0f);
		m_WaterTint = CColor(0.28f, 0.3f, 0.59f, 1.0f);
		m_Murkiness = 0.45f;
		m_WaterType = L"ocean";
	}

	void SetMapSize(ssize_t size)
	{
		m_MapSize = size;
		m_WaterRData.SetMapSize(size);
		// No need to mark the terrain as dirty, we can assume that's already been done.
	}

	void SetWaterHeight(float height)
	{
		m_WaterHeight = height;
		m_WaterRData.SetDirty();
		m_Terrain.MakeDirty(RENDERDATA_UPDATE_VERTICES);
	}

	float GetWaterHeight() const { return m_WaterHeight; }

	WaterRData& GetRData() { return m_WaterRData; }

	void RecomputeWaterData() { m_WaterRData.SetDirty(); }

protected:
	CTerrain& m_Terrain;

	float m_WaterHeight;

	ssize_t m_MapSize;

public:
	// Water parameters - public for convenience in Atlas
	// TODO: ideally these would be wrapped somehow.

	// Which texture to use.
	std::wstring m_WaterType;
	// Color of the water without refractions. This is what you're seeing when the water's deep or murkiness high.
	CColor m_WaterColor;
	// Tint of refraction in the water.
	CColor m_WaterTint;
	// How big the waves are.
	float m_Waviness;
	// How murky the water is.
	float m_Murkiness;
	// In which direction the water waves go.
	float m_WindAngle;

protected:

	WaterRData m_WaterRData;
};


#endif // INCLUDED_WATER
