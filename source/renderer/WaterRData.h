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

#ifndef INCLUDED_WATERRDATA
#define INCLUDED_WATERRDATA

#include "graphics/Color.h"
#include "graphics/Texture.h"
#include "maths/Matrix3D.h"
#include "maths/Vector2D.h"
#include "renderer/backend/gl/DeviceCommandContext.h"
#include "renderer/backend/gl/Texture.h"
#include "renderer/VertexBufferManager.h"

#include <memory>
#include <vector>

class CFrustum;
class CWater;

struct WaveObject;

/**
 * Rendering data for water.
 */
class WaterRData
{
public:
	WaterRData(const CWater& water);
	~WaterRData();

	void SetDirty();

	void SetMapSize(size_t size);

	const float* GetWindStrength()
	{
		if (m_NeedInfoUpdate)
			RecomputeWaterData();
		return m_WindStrength.get();
	}

	const std::vector<std::unique_ptr<WaveObject>>& GetShoreWavesData()
	{
		if (m_NeedInfoUpdate)
			RecomputeWaterData();
		return m_ShoreWaves;
	}

	const CVertexBufferManager::Handle& GetShoreWavesIndices()
	{
		if (m_NeedInfoUpdate)
			RecomputeWaterData();
		return m_ShoreWavesVBIndices;
	}

protected:
	/**
	 * RecomputeWaterData: calculates all derived data from the waterheight
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

	/**
	 * CreateWaveMeshes: Creates the waves objects (and meshes).
	 */
	void CreateWaveMeshes();

protected:
	const CWater& m_Water;

	// How strong the waves are at point X. % of waviness.
	std::unique_ptr<float[]> m_WindStrength;
	// How far from the shore a point is. Manhattan.
	std::unique_ptr<float[]> m_DistanceHeightmap;

	// Waves vertex buffers
	// TODO: measure storing value instead of pointer.
	std::vector<std::unique_ptr<WaveObject>> m_ShoreWaves;
	// Waves indices buffer. Only one since All Wave Objects have the same.
	CVertexBufferManager::Handle m_ShoreWavesVBIndices;

	size_t m_MapSize;

	// requires also recreating the super fancy information.
	bool m_NeedInfoUpdate;
};


#endif // INCLUDED_WATERRDATA
