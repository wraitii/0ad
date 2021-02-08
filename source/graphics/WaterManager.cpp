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

#include "precompiled.h"

#include "WaterManager.h"

#include "graphics/RenderableObject.h"
#include "graphics/Terrain.h"
#include "graphics/TextureManager.h"
#include "maths/Vector2D.h"
#include "ps/Game.h"
#include "ps/World.h"
#include "renderer/Renderer.h"
#include "renderer/WaterRendering.h"

WaterManager::WaterManager()
{
	m_WaterHeight = 5.0f;

	m_WindAngle = 0.0f;
	m_Waviness = 8.0f;
	m_WaterColor = CColor(0.3f, 0.35f, 0.7f, 1.0f);
	m_WaterTint = CColor(0.28f, 0.3f, 0.59f, 1.0f);
	m_Murkiness = 0.45f;

	m_DistanceHeightmap = NULL;
	m_BlurredNormalMap = NULL;
	m_WindStrength = NULL;

	m_WaterType = L"ocean";

	m_MapSize = 0;

	if (CRenderer::IsInitialised())
	{
		g_Renderer.GetWaterRendering().m_WaterManager = this;
		// Load textures in the renderer.
		// TODO: would probably be better to do this elsewhere,
		// but I haven't actually found a more convenient place.
		// Putting these here avoids doing it until GameView is first created,
		// which only happens on the first game start.
		g_Renderer.GetWaterRendering().PrepareTextures();
		g_Renderer.GetWaterRendering().LoadNormalTextures(m_WaterType);
	}

	m_RenderWater = false;
	m_NeedRecomputation = true;
}

WaterManager::~WaterManager()
{
	if (CRenderer::IsInitialised())
		g_Renderer.GetWaterRendering().m_WaterManager = nullptr;

	delete[] m_WindStrength;
	delete[] m_DistanceHeightmap;
	delete[] m_BlurredNormalMap;
}

void WaterManager::SetRenderingEnabled(bool enabled)
{
	if (enabled == m_RenderWater)
		return;
	m_RenderWater = enabled;
	if (enabled)
		m_NeedRecomputation = true;
}

void WaterManager::SetWaterType(std::wstring type)
{
	if (type == m_WaterType)
		return;
	m_WaterType = type;

	if (CRenderer::IsInitialised())
		g_Renderer.GetWaterRendering().LoadNormalTextures(m_WaterType);
}

void WaterManager::SetWaterHeight(float height)
{
	if (height == m_WaterHeight)
		return;

	m_WaterHeight = height;

	m_NeedRecomputation = true;
}

void WaterManager::SetMapSize(size_t size)
{
	if (size == m_MapSize)
		return;
	m_MapSize = size;

	m_NeedRecomputation = true;
}

void WaterManager::MarkDirty(i32, i32, i32, i32)
{
	// We need to update vertices once we've recomputed data, so for now just mark for recomputation.
	m_NeedRecomputation = true;
}

void WaterManager::RecomputeWaterData()
{
	LOGWARNING("RecomputeWaterData1");
	m_NeedRecomputation = false;

	if (!m_MapSize)
		return;

	LOGWARNING("RecomputeWaterData2");

	RecomputeDistanceHeightmap();
	RecomputeWindStrength();
	if (CRenderer::IsInitialised())
		g_Renderer.GetWaterRendering().CreateWaveMeshes();

	// PatchRData contain the water data, so we need to rebuild those.
	g_Game->GetWorld()->GetTerrain()->MakeDirty(0, 0, m_MapSize, m_MapSize, RENDERDATA_UPDATE_VERTICES);
}

float WaterManager::GetWindStrength(int x, int z) const
{
	if (!m_WindStrength)
		return 0;
	return m_WindStrength[x + m_MapSize * z];
}

template<bool Transpose>
static inline void ComputeDirection(float* distanceMap, const u16* heightmap, float waterHeight, size_t SideSize, size_t maxLevel)
{
#define ABOVEWATER(x, z) (HEIGHT_SCALE * heightmap[z*SideSize + x] >= waterHeight)
#define UPDATELOOKAHEAD \
for (; lookahead <= id2+maxLevel && lookahead < SideSize && \
((!Transpose && !ABOVEWATER(lookahead, id1)) || (Transpose && !ABOVEWATER(id1, lookahead))); ++lookahead)
	// Algorithm:
	// We want to know the distance to the closest shore point. Go through each line/column,
	// keep track of when we encountered the last shore point and how far ahead the next one is.
	for (size_t id1 = 0; id1 < SideSize; ++id1)
	{
		size_t id2 = 0;
		const size_t& x = Transpose ? id1 : id2;
		const size_t& z = Transpose ? id2 : id1;

		size_t level = ABOVEWATER(x, z) ? 0 : maxLevel;
		size_t lookahead = (size_t)(level > 0);

		UPDATELOOKAHEAD;

		// start moving
		for (; id2 < SideSize; ++id2)
		{
			// update current level
			if (ABOVEWATER(x, z))
				level = 0;
			else
				level = std::min(level+1, maxLevel);

			// move lookahead
			if (lookahead == id2)
				++lookahead;
			UPDATELOOKAHEAD;

			// This is the important bit: set the distance to either:
			// - the distance to the previous shore point (level)
			// - the distance to the next shore point (lookahead-id2)
			distanceMap[z*SideSize + x] = std::min(distanceMap[z*SideSize + x], (float)std::min(lookahead-id2, level));
		}
	}
#undef ABOVEWATER
#undef UPDATELOOKAHEAD
}

///////////////////////////////////////////////////////////////////
// Calculate our binary heightmap from the terrain heightmap.
void WaterManager::RecomputeDistanceHeightmap()
{
	CTerrain* terrain = g_Game->GetWorld()->GetTerrain();
	if (!terrain || !terrain->GetHeightMap())
		return;

	size_t SideSize = m_MapSize;

	// we want to look ahead some distance, but not too much (less efficient and not interesting). This is our lookahead.
	const size_t maxLevel = 5;

	if (m_DistanceHeightmap == NULL)
	{
		m_DistanceHeightmap = new float[SideSize*SideSize];
		std::fill(m_DistanceHeightmap, m_DistanceHeightmap + SideSize*SideSize, (float)maxLevel);
	}

	// Create a manhattan-distance heightmap.
	// This could be refined to only be done near the coast itself, but it's probably not necessary.

	u16* heightmap = terrain->GetHeightMap();

	ComputeDirection<false>(m_DistanceHeightmap, heightmap, m_WaterHeight, SideSize, maxLevel);
	ComputeDirection<true>(m_DistanceHeightmap, heightmap, m_WaterHeight, SideSize, maxLevel);
}

///////////////////////////////////////////////////////////////////
// Calculate the strength of the wind at a given point on the map.
void WaterManager::RecomputeWindStrength()
{
	if (m_MapSize <= 0)
		return;

	if (m_WindStrength == nullptr)
		m_WindStrength = new float[m_MapSize*m_MapSize];

	CTerrain* terrain = g_Game->GetWorld()->GetTerrain();
	if (!terrain || !terrain->GetHeightMap())
		return;

	CVector2D windDir = CVector2D(cos(m_WindAngle), sin(m_WindAngle));

	int stepSize = 10;
	ssize_t windX = -round(stepSize * windDir.X);
	ssize_t windY = -round(stepSize * windDir.Y);

	struct SWindPoint {
		SWindPoint(size_t x, size_t y, float strength) : X(x), Y(y), windStrength(strength) {}
		ssize_t X;
		ssize_t Y;
		float windStrength;
	};

	std::vector<SWindPoint> startingPoints;
	std::vector<std::pair<int, int>> movement; // Every increment, move each starting point by all of these.

	// Compute starting points (one or two edges of the map) and how much to move each computation increment.
	if (fabs(windDir.X) < 0.01f)
	{
		movement.emplace_back(0, windY > 0.f ? 1 : -1);
		startingPoints.reserve(m_MapSize);
		size_t start = windY > 0 ? 0 : m_MapSize - 1;
		for (size_t x = 0; x < m_MapSize; ++x)
			startingPoints.emplace_back(x, start, 0.f);
	}
	else if (fabs(windDir.Y) < 0.01f)
	{
		movement.emplace_back(windX > 0.f ? 1 : - 1, 0);
		size_t start = windX > 0 ? 0 : m_MapSize - 1;
		for (size_t z = 0; z < m_MapSize; ++z)
			startingPoints.emplace_back(start, z, 0.f);
	}
	else
	{
		startingPoints.reserve(m_MapSize * 2);
		// Points along X.
		size_t start = windY > 0 ? 0 : m_MapSize - 1;
		for (size_t x = 0; x < m_MapSize; ++x)
			startingPoints.emplace_back(x, start, 0.f);
		// Points along Z, avoid repeating the corner point.
		start = windX > 0 ? 0 : m_MapSize - 1;
		if (windY > 0)
			for (size_t z = 1; z < m_MapSize; ++z)
				startingPoints.emplace_back(start, z, 0.f);
		else
			for (size_t z = 0; z < m_MapSize-1; ++z)
				startingPoints.emplace_back(start, z, 0.f);

		// Compute movement array.
		movement.reserve(std::max(std::abs(windX),std::abs(windY)));
		while (windX != 0 || windY != 0)
		{
			std::pair<ssize_t, ssize_t> move = {
				windX == 0 ? 0 : windX > 0 ? +1 : -1,
				windY == 0 ? 0 : windY > 0 ? +1 : -1
			};
			windX -= move.first;
			windY -= move.second;
			movement.push_back(move);
		}
	}

	// We have all starting points ready, move them all until the map is covered.
	for (SWindPoint& point : startingPoints)
	{
		// Starting velocity is 1.0 unless in shallow water.
		m_WindStrength[point.Y * m_MapSize + point.X] = 1.f;
		float depth = m_WaterHeight - terrain->GetVertexGroundLevel(point.X, point.Y);
		if (depth > 0.f && depth < 2.f)
			m_WindStrength[point.Y * m_MapSize + point.X] = depth / 2.f;
		point.windStrength = m_WindStrength[point.Y * m_MapSize + point.X];

		bool onMap = true;
		while (onMap)
			for (size_t step = 0; step < movement.size(); ++step)
			{
				// Move wind speed towards the mean.
				point.windStrength = 0.15f + point.windStrength * 0.85f;

				// Adjust speed based on height difference, a positive height difference slowly increases speed (simulate venturi effect)
				// and a lower height reduces speed (wind protection from hills/...)
				float heightDiff = std::max(m_WaterHeight, terrain->GetVertexGroundLevel(point.X + movement[step].first, point.Y + movement[step].second)) -
				std::max(m_WaterHeight, terrain->GetVertexGroundLevel(point.X, point.Y));
				if (heightDiff > 0.f)
					point.windStrength = std::min(2.f, point.windStrength + std::min(4.f, heightDiff) / 40.f);
				else
					point.windStrength = std::max(0.f, point.windStrength + std::max(-4.f, heightDiff) / 5.f);

				point.X += movement[step].first;
				point.Y += movement[step].second;

				if (point.X < 0 || point.X >= static_cast<ssize_t>(m_MapSize) || point.Y < 0 || point.Y >= static_cast<ssize_t>(m_MapSize))
				{
					onMap = false;
					break;
				}
				m_WindStrength[point.Y * m_MapSize + point.X] = point.windStrength;
			}
	}
	// TODO: should perhaps blur a little, or change the above code to incorporate neighboring tiles a bit.
}

