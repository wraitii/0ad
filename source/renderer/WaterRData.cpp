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

#include "renderer/WaterRData.h"

#include "graphics/Water.h"
#include "maths/MathUtil.h"
#include "ps/Game.h"
#include "ps/World.h"
#include "renderer/Renderer.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <set>
#include <utility>
#include <vector>

struct CoastalPoint
{
	CoastalPoint(int idx, CVector2D pos) : index(idx), position(pos) {};
	int index;
	CVector2D position;
};

struct SWavesVertex
{
	// vertex position
	CVector3D m_BasePosition;
	CVector3D m_ApexPosition;
	CVector3D m_SplashPosition;
	CVector3D m_RetreatPosition;

	CVector2D m_PerpVect;
	u8 m_UV[3];

	// pad to a power of two
	u8 m_Padding[5];
};
cassert(sizeof(SWavesVertex) == 64);

struct WaveObject
{
	CVertexBufferManager::Handle m_VBVertices;
	CBoundingBoxAligned m_AABB;
	size_t m_Width;
	float m_TimeDiff;
};

WaterRData::WaterRData(const CWater& water) : m_Water(water)
{
	m_NeedInfoUpdate = true;

	m_MapSize = 0;
}

WaterRData::~WaterRData()
{
	m_ShoreWaves.clear();
	m_ShoreWavesVBIndices.Reset();

	m_DistanceHeightmap.reset();
	m_WindStrength.reset();
}

void WaterRData::SetDirty()
{
	m_NeedInfoUpdate = true;
	m_DistanceHeightmap.reset();
	m_WindStrength.reset();
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
void WaterRData::RecomputeDistanceHeightmap()
{
	CTerrain* terrain = g_Game->GetWorld()->GetTerrain();
	if (!terrain || !terrain->GetHeightMap())
		return;

	size_t SideSize = m_MapSize;

	// we want to look ahead some distance, but not too much (less efficient and not interesting). This is our lookahead.
	const size_t maxLevel = 5;

	if (!m_DistanceHeightmap)
	{
		m_DistanceHeightmap = std::make_unique<float[]>(SideSize * SideSize);
		std::fill(m_DistanceHeightmap.get(), m_DistanceHeightmap.get() + SideSize * SideSize, static_cast<float>(maxLevel));
	}

	// Create a manhattan-distance heightmap.
	// This could be refined to only be done near the coast itself, but it's probably not necessary.

	u16* heightmap = terrain->GetHeightMap();

	ComputeDirection<false>(m_DistanceHeightmap.get(), heightmap, m_Water.GetWaterHeight(), SideSize, maxLevel);
	ComputeDirection<true>(m_DistanceHeightmap.get(), heightmap, m_Water.GetWaterHeight(), SideSize, maxLevel);
}

// This requires m_DistanceHeightmap to be defined properly.
void WaterRData::CreateWaveMeshes()
{
	if (m_MapSize == 0)
		return;

	CTerrain* terrain = g_Game->GetWorld()->GetTerrain();
	if (!terrain || !terrain->GetHeightMap())
		return;

	m_ShoreWaves.clear();
	m_ShoreWavesVBIndices.Reset();

	if (m_Water.m_Waviness < 5.0f && m_Water.m_WaterType != L"ocean")
		return;

	size_t SideSize = m_MapSize;

	// First step: get the points near the coast.
	std::set<int> CoastalPointsSet;
	for (size_t z = 1; z < SideSize-1; ++z)
		for (size_t x = 1; x < SideSize-1; ++x)
			// get the points not on the shore but near it, ocean-side
			if (m_DistanceHeightmap[z*m_MapSize + x] > 0.5f && m_DistanceHeightmap[z*m_MapSize + x] < 1.5f)
				CoastalPointsSet.insert((z)*SideSize + x);

	// Second step: create chains out of those coastal points.
	static const int around[8][2] = { { -1,-1 }, { -1,0 }, { -1,1 }, { 0,1 }, { 1,1 }, { 1,0 }, { 1,-1 }, { 0,-1 } };

	std::vector<std::deque<CoastalPoint> > CoastalPointsChains;
	while (!CoastalPointsSet.empty())
	{
		int index = *(CoastalPointsSet.begin());
		int x = index % SideSize;
		int y = (index - x ) / SideSize;

		std::deque<CoastalPoint> Chain;

		Chain.push_front(CoastalPoint(index,CVector2D(x*4,y*4)));

		// Erase us.
		CoastalPointsSet.erase(CoastalPointsSet.begin());

		// We're our starter points. At most we can have 2 points close to us.
		// We'll pick the first one and look for its neighbors (he can only have one new)
		// Up until we either reach the end of the chain, or ourselves.
		// Then go down the other direction if there is any.
		int neighbours[2] = { -1, -1 };
		int nbNeighb = 0;
		for (int i = 0; i < 8; ++i)
		{
			if (CoastalPointsSet.count(x + around[i][0] + (y + around[i][1])*SideSize))
			{
				if (nbNeighb < 2)
					neighbours[nbNeighb] = x + around[i][0] + (y + around[i][1])*SideSize;
				++nbNeighb;
			}
		}
		if (nbNeighb > 2)
			continue;

		for (int i = 0; i < 2; ++i)
		{
			if (neighbours[i] == -1)
				continue;
			// Move to our neighboring point
			int xx = neighbours[i] % SideSize;
			int yy = (neighbours[i] - xx ) / SideSize;
			int indexx = xx + yy*SideSize;
			int endedChain = false;

			if (i == 0)
				Chain.push_back(CoastalPoint(indexx,CVector2D(xx*4,yy*4)));
			else
				Chain.push_front(CoastalPoint(indexx,CVector2D(xx*4,yy*4)));

			// If there's a loop we'll be the "other" neighboring point already so check for that.
			// We'll readd at the end/front the other one to have full squares.
			if (CoastalPointsSet.count(indexx) == 0)
				break;

			CoastalPointsSet.erase(indexx);

			// Start checking from there.
			while(!endedChain)
			{
				bool found = false;
				nbNeighb = 0;
				for (int p = 0; p < 8; ++p)
				{
					if (CoastalPointsSet.count(xx+around[p][0] + (yy + around[p][1])*SideSize))
					{
						if (nbNeighb >= 2)
						{
							CoastalPointsSet.erase(xx + yy*SideSize);
							continue;
						}
						++nbNeighb;
						// We've found a new point around us.
						// Move there
						xx = xx + around[p][0];
						yy = yy + around[p][1];
						indexx = xx + yy*SideSize;
						if (i == 0)
							Chain.push_back(CoastalPoint(indexx,CVector2D(xx*4,yy*4)));
						else
							Chain.push_front(CoastalPoint(indexx,CVector2D(xx*4,yy*4)));
						CoastalPointsSet.erase(xx + yy*SideSize);
						found = true;
						break;
					}
				}
				if (!found)
					endedChain = true;
			}
		}
		if (Chain.size() > 10)
			CoastalPointsChains.push_back(Chain);
	}

	// (optional) third step: Smooth chains out.
	// This is also really dumb.
	for (size_t i = 0; i < CoastalPointsChains.size(); ++i)
	{
		// Bump 1 for smoother.
		for (int p = 0; p < 3; ++p)
		{
			for (size_t j = 1; j < CoastalPointsChains[i].size()-1; ++j)
			{
				CVector2D realPos = CoastalPointsChains[i][j-1].position + CoastalPointsChains[i][j+1].position;

				CoastalPointsChains[i][j].position = (CoastalPointsChains[i][j].position + realPos/2.0f)/2.0f;
			}
		}
	}

	// Fourth step: create waves themselves, using those chains. We basically create subchains.
	u16 waveSizes = 14;	// maximal size in width.

	// Construct indices buffer (we can afford one for all of them)
	std::vector<u16> water_indices;
	for (u16 a = 0; a < waveSizes - 1; ++a)
	{
		for (u16 rect = 0; rect < 7; ++rect)
		{
			water_indices.push_back(a * 9 + rect);
			water_indices.push_back(a * 9 + 9 + rect);
			water_indices.push_back(a * 9 + 1 + rect);
			water_indices.push_back(a * 9 + 9 + rect);
			water_indices.push_back(a * 9 + 10 + rect);
			water_indices.push_back(a * 9 + 1 + rect);
		}
	}
	// Generic indexes, max-length
	// Generic indexes, max-length
	m_ShoreWavesVBIndices = g_VBMan.AllocateChunk(
		sizeof(u16), water_indices.size(),
		Renderer::Backend::IBuffer::Type::INDEX, false,
		nullptr, CVertexBufferManager::Group::WATER);
	m_ShoreWavesVBIndices->m_Owner->UpdateChunkVertices(m_ShoreWavesVBIndices.Get(), &water_indices[0]);

	float diff = (rand() % 50) / 5.0f;

	const float waterHeight = m_Water.GetWaterHeight();

	std::vector<SWavesVertex> vertices, reversed;
	for (size_t i = 0; i < CoastalPointsChains.size(); ++i)
	{
		for (size_t j = 0; j < CoastalPointsChains[i].size()-waveSizes; ++j)
		{
			if (CoastalPointsChains[i].size()- 1 - j < waveSizes)
				break;

			u16 width = waveSizes;

			// First pass to get some parameters out.
			float outmost = 0.0f;	// how far to move on the shore.
			float avgDepth = 0.0f;
			int sign = 1;
			CVector2D firstPerp(0,0), perp(0,0), lastPerp(0,0);
			for (u16 a = 0; a < waveSizes;++a)
			{
				lastPerp = perp;
				perp = CVector2D(0,0);
				int nb = 0;
				CVector2D pos = CoastalPointsChains[i][j+a].position;
				CVector2D posPlus;
				CVector2D posMinus;
				if (a > 0)
				{
					++nb;
					posMinus = CoastalPointsChains[i][j+a-1].position;
					perp += pos-posMinus;
				}
				if (a < waveSizes-1)
				{
					++nb;
					posPlus = CoastalPointsChains[i][j+a+1].position;
					perp += posPlus-pos;
				}
				perp /= nb;
				perp = CVector2D(-perp.Y,perp.X).Normalized();

				if (a == 0)
					firstPerp = perp;

				if ( a > 1 && perp.Dot(lastPerp) < 0.90f && perp.Dot(firstPerp) < 0.70f)
				{
					width = a+1;
					break;
				}

				if (terrain->GetExactGroundLevel(pos.X+perp.X*1.5f, pos.Y+perp.Y*1.5f) > waterHeight)
					sign = -1;

				avgDepth += terrain->GetExactGroundLevel(pos.X+sign*perp.X*20.0f, pos.Y+sign*perp.Y*20.0f) - waterHeight;

				float localOutmost = -2.0f;
				while (localOutmost < 0.0f)
				{
					float depth = terrain->GetExactGroundLevel(pos.X+sign*perp.X*localOutmost, pos.Y+sign*perp.Y*localOutmost) - waterHeight;
					if (depth < 0.0f || depth > 0.6f)
						localOutmost += 0.2f;
					else
						break;
				}

				outmost += localOutmost;
			}
			if (width < 5)
			{
				j += 6;
				continue;
			}

			outmost /= width;

			if (outmost > -0.5f)
			{
				j += 3;
				continue;
			}
			outmost = -2.5f + outmost * m_Water.m_Waviness/10.0f;

			avgDepth /= width;

			if (avgDepth > -1.3f)
			{
				j += 3;
				continue;
			}
			// we passed the checks, we can create a wave of size "width".

			std::unique_ptr<WaveObject> shoreWave = std::make_unique<WaveObject>();
			vertices.clear();
			vertices.reserve(9 * width);

			shoreWave->m_Width = width;
			shoreWave->m_TimeDiff = diff;
			diff += (rand() % 100) / 25.0f + 4.0f;

			for (u16 a = 0; a < width;++a)
			{
				perp = CVector2D(0,0);
				int nb = 0;
				CVector2D pos = CoastalPointsChains[i][j+a].position;
				CVector2D posPlus;
				CVector2D posMinus;
				if (a > 0)
				{
					++nb;
					posMinus = CoastalPointsChains[i][j+a-1].position;
					perp += pos-posMinus;
				}
				if (a < waveSizes-1)
				{
					++nb;
					posPlus = CoastalPointsChains[i][j+a+1].position;
					perp += posPlus-pos;
				}
				perp /= nb;
				perp = CVector2D(-perp.Y,perp.X).Normalized();

				SWavesVertex point[9];

				float baseHeight = 0.04f;

				float halfWidth = (width-1.0f)/2.0f;
				float sideNess = sqrtf(Clamp( (halfWidth - fabsf(a - halfWidth)) / 3.0f, 0.0f, 1.0f));

				point[0].m_UV[0] = a; point[0].m_UV[1] = 8;
				point[1].m_UV[0] = a; point[1].m_UV[1] = 7;
				point[2].m_UV[0] = a; point[2].m_UV[1] = 6;
				point[3].m_UV[0] = a; point[3].m_UV[1] = 5;
				point[4].m_UV[0] = a; point[4].m_UV[1] = 4;
				point[5].m_UV[0] = a; point[5].m_UV[1] = 3;
				point[6].m_UV[0] = a; point[6].m_UV[1] = 2;
				point[7].m_UV[0] = a; point[7].m_UV[1] = 1;
				point[8].m_UV[0] = a; point[8].m_UV[1] = 0;

				point[0].m_PerpVect = perp;
				point[1].m_PerpVect = perp;
				point[2].m_PerpVect = perp;
				point[3].m_PerpVect = perp;
				point[4].m_PerpVect = perp;
				point[5].m_PerpVect = perp;
				point[6].m_PerpVect = perp;
				point[7].m_PerpVect = perp;
				point[8].m_PerpVect = perp;

				static const float perpT1[9] = { 6.0f, 6.05f, 6.1f, 6.2f, 6.3f, 6.4f, 6.5f, 6.6f, 9.7f };
				static const float perpT2[9] = { 2.0f, 2.1f,  2.2f, 2.3f, 2.4f, 3.0f, 3.3f, 3.6f, 9.5f };
				static const float perpT3[9] = { 1.1f, 0.7f, -0.2f, 0.0f, 0.6f, 1.3f, 2.2f, 3.6f, 9.0f };
				static const float perpT4[9] = { 2.0f, 2.1f,  1.2f, 1.5f, 1.7f, 1.9f, 2.7f, 3.8f, 9.0f };

				static const float heightT1[9] = { 0.0f, 0.2f, 0.5f, 0.8f, 0.9f, 0.85f, 0.6f, 0.2f, 0.0 };
				static const float heightT2[9] = { -0.8f, -0.4f, 0.0f, 0.1f, 0.1f, 0.03f, 0.0f, 0.0f, 0.0 };
				static const float heightT3[9] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };

				for (size_t t = 0; t < 9; ++t)
				{
					float terrHeight = 0.05f + terrain->GetExactGroundLevel(pos.X+sign*perp.X*(perpT1[t]+outmost),
																			pos.Y+sign*perp.Y*(perpT1[t]+outmost));
					point[t].m_BasePosition = CVector3D(pos.X+sign*perp.X*(perpT1[t]+outmost), baseHeight + heightT1[t]*sideNess + std::max(waterHeight,terrHeight),
														pos.Y+sign*perp.Y*(perpT1[t]+outmost));
				}
				for (size_t t = 0; t < 9; ++t)
				{
					float terrHeight = 0.05f + terrain->GetExactGroundLevel(pos.X+sign*perp.X*(perpT2[t]+outmost),
																			pos.Y+sign*perp.Y*(perpT2[t]+outmost));
					point[t].m_ApexPosition = CVector3D(pos.X+sign*perp.X*(perpT2[t]+outmost), baseHeight + heightT1[t]*sideNess + std::max(waterHeight,terrHeight),
														pos.Y+sign*perp.Y*(perpT2[t]+outmost));
				}
				for (size_t t = 0; t < 9; ++t)
				{
					float terrHeight = 0.05f + terrain->GetExactGroundLevel(pos.X+sign*perp.X*(perpT3[t]+outmost*sideNess),
																			pos.Y+sign*perp.Y*(perpT3[t]+outmost*sideNess));
					point[t].m_SplashPosition = CVector3D(pos.X+sign*perp.X*(perpT3[t]+outmost*sideNess), baseHeight + heightT2[t]*sideNess + std::max(waterHeight,terrHeight), pos.Y+sign*perp.Y*(perpT3[t]+outmost*sideNess));
				}
				for (size_t t = 0; t < 9; ++t)
				{
					float terrHeight = 0.05f + terrain->GetExactGroundLevel(pos.X+sign*perp.X*(perpT4[t]+outmost),
																			pos.Y+sign*perp.Y*(perpT4[t]+outmost));
					point[t].m_RetreatPosition = CVector3D(pos.X+sign*perp.X*(perpT4[t]+outmost), baseHeight + heightT3[t]*sideNess + std::max(waterHeight,terrHeight),
														   pos.Y+sign*perp.Y*(perpT4[t]+outmost));
				}

				vertices.push_back(point[8]);
				vertices.push_back(point[7]);
				vertices.push_back(point[6]);
				vertices.push_back(point[5]);
				vertices.push_back(point[4]);
				vertices.push_back(point[3]);
				vertices.push_back(point[2]);
				vertices.push_back(point[1]);
				vertices.push_back(point[0]);

				shoreWave->m_AABB += point[8].m_SplashPosition;
				shoreWave->m_AABB += point[8].m_BasePosition;
				shoreWave->m_AABB += point[0].m_SplashPosition;
				shoreWave->m_AABB += point[0].m_BasePosition;
				shoreWave->m_AABB += point[4].m_ApexPosition;
			}

			if (sign == 1)
			{
				// Let's do some fancy reversing.
				reversed.clear();
				reversed.reserve(vertices.size());
				for (int a = width - 1; a >= 0; --a)
				{
					for (size_t t = 0; t < 9; ++t)
						reversed.push_back(vertices[a * 9 + t]);
				}
				std::swap(vertices, reversed);
			}
			j += width/2-1;

			shoreWave->m_VBVertices = g_VBMan.AllocateChunk(
				sizeof(SWavesVertex), vertices.size(),
				Renderer::Backend::IBuffer::Type::VERTEX, false,
				nullptr, CVertexBufferManager::Group::WATER);
			shoreWave->m_VBVertices->m_Owner->UpdateChunkVertices(shoreWave->m_VBVertices.Get(), &vertices[0]);

			m_ShoreWaves.emplace_back(std::move(shoreWave));
		}
	}
}

void WaterRData::RecomputeWaterData()
{
	if (!m_MapSize)
		return;

	RecomputeDistanceHeightmap();
	RecomputeWindStrength();
	CreateWaveMeshes();
}

///////////////////////////////////////////////////////////////////
// Calculate the strength of the wind at a given point on the map.
void WaterRData::RecomputeWindStrength()
{
	if (m_MapSize <= 0)
		return;

	if (!m_WindStrength)
		m_WindStrength = std::make_unique<float[]>(m_MapSize * m_MapSize);

	CTerrain* terrain = g_Game->GetWorld()->GetTerrain();
	if (!terrain || !terrain->GetHeightMap())
		return;

	CVector2D windDir = CVector2D(cos(m_Water.m_WindAngle), sin(m_Water.m_WindAngle));

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
		startingPoints.reserve(m_MapSize);
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

	const float waterHeight = m_Water.GetWaterHeight();

	// We have all starting points ready, move them all until the map is covered.
	for (SWindPoint& point : startingPoints)
	{
		// Starting velocity is 1.0 unless in shallow water.
		m_WindStrength[point.Y * m_MapSize + point.X] = 1.f;
		float depth = waterHeight - terrain->GetVertexGroundLevel(point.X, point.Y);
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
				float heightDiff = std::max(waterHeight, terrain->GetVertexGroundLevel(point.X + movement[step].first, point.Y + movement[step].second)) -
					std::max(waterHeight, terrain->GetVertexGroundLevel(point.X, point.Y));
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

////////////////////////////////////////////////////////////////////////
// TODO: This will always recalculate for now
void WaterRData::SetMapSize(size_t size)
{
	m_MapSize = size;
	SetDirty();
}
