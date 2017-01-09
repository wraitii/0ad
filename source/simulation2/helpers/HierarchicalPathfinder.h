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

#ifndef INCLUDED_HIERPATHFINDER
#define INCLUDED_HIERPATHFINDER

#include "Pathfinding.h"

#include "renderer/TerrainOverlay.h"
#include "Render.h"
#include "graphics/SColor.h"

#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>

/**
 * Hierarchical pathfinder.
 *
 * Deals with connectivity (can point A reach point B?)
 *
 * The navcell-grid representation of the map is split into fixed-size chunks.
 * Within a chunk, each maximal set of adjacently-connected passable navcells
 * is defined as a region.
 * Each region is a vertex in the hierarchical pathfinder's graph.
 * When two regions in adjacent chunks are connected by passable navcells,
 * the graph contains an edge between the corresponding two vertexes.
 * (by design, there can never be an edge between two regions in the same chunk.)
 *
 * Those fixed-size chunks are used to efficiently compute "global regions" by effectively flood-filling.
 * Those can then be used to immediately determine if two reachables points are connected
 *
 * The main use of this class is to convert an arbitrary PathGoal to a reachable navcell
 * This happens in MakeGoalReachable, which implements A* over the chunks.
 * Currently, the resulting path is unused.
 *
 */

#ifdef TEST
class TestCmpPathfinder;
#endif

class HierarchicalOverlay;

class HierarchicalPathfinder
{
#ifdef TEST
	friend class TestCmpPathfinder;
#endif
public:
	typedef u32 GlobalRegionID;

	struct RegionID
	{
		u8 ci, cj; // chunk ID
		u16 r; // unique-per-chunk local region ID

		RegionID(u8 ci, u8 cj, u16 r) : ci(ci), cj(cj), r(r) { }

		bool operator<(const RegionID& b) const
		{
			// Sort by chunk ID, then by per-chunk region ID
			if (ci < b.ci)
				return true;
			if (b.ci < ci)
				return false;
			if (cj < b.cj)
				return true;
			if (b.cj < cj)
				return false;
			return r < b.r;
		}

		bool operator==(const RegionID& b) const
		{
			return ((ci == b.ci) && (cj == b.cj) && (r == b.r));
		}
	};

	HierarchicalPathfinder();
	~HierarchicalPathfinder();

	void SetDebugOverlay(bool enabled, const CSimContext* simContext);

	// Non-pathfinding grids will never be recomputed on calling HierarchicalPathfinder::Update
	void Recompute(Grid<NavcellData>* passabilityGrid,
		const std::map<std::string, pass_class_t>& nonPathfindingPassClassMasks,
		const std::map<std::string, pass_class_t>& pathfindingPassClassMasks);

	void Update(Grid<NavcellData>* grid, const Grid<u8>& dirtinessGrid);

	bool IsChunkDirty(int ci, int cj, const Grid<u8>& dirtinessGrid) const;

	RegionID Get(u16 i, u16 j, pass_class_t passClass);
	GlobalRegionID GetGlobalRegion(u16 i, u16 j, pass_class_t passClass);

	/**
	 * Updates @p goal to a point goal guaranteed to be reachable from the original navcell
	 * @p i0, @p j0 (which is assumed to be on a passable navcell).
	 *
	 * If the goal is not reachable, it is replaced with an acceptable point goal
	 * This function does not necessarily return the closest navcell to the goal
	 * but the one with the lowest f score of the A* algorithm.
	 * This means it is usually a tradeoff between walking time and distance to the goal.
	 *
	 * In the case of a non-point reachable goal, it is replaced with a point goal
	 * at the reachable navcell of the goal which is nearest to the starting navcell.
	 * TODO: since A* is used, it could return the reachable navcell nearest to the penultimate region visited.
	 * which is probably better (imagine a path that must bend around).
	 */
	void MakeGoalReachable(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass);

	/**
	 * Updates @p i, @p j (which is assumed to be an impassable navcell)
	 * to the nearest passable navcell.
	 */
	void FindNearestPassableNavcell(u16& i, u16& j, pass_class_t passClass);

	/**
	 * Generates the connectivity grid associated with the given pass_class
	 */
	Grid<u16> GetConnectivityGrid(pass_class_t passClass);

	pass_class_t GetPassabilityClass(const std::string& name) const
	{
		auto it = m_PassClassMasks.find(name);
		if (it != m_PassClassMasks.end())
			return it->second;

		LOGERROR("Invalid passability class name '%s'", name.c_str());
		return 0;
	}

private:
	static const u8 CHUNK_SIZE = 96; // number of navcells per side
									 // TODO: figure out best number. Probably 64 < n < 128

	struct Chunk
	{
		u8 m_ChunkI, m_ChunkJ; // chunk ID
		std::vector<u16> m_RegionsID; // IDs of local region, without 0
		u16 m_Regions[CHUNK_SIZE][CHUNK_SIZE]; // local region ID per navcell

		cassert(CHUNK_SIZE*CHUNK_SIZE/2 < 65536); // otherwise we could overflow m_NumRegions with a checkerboard pattern

		void InitRegions(int ci, int cj, Grid<NavcellData>* grid, pass_class_t passClass);

		RegionID Get(int i, int j) const;

		void RegionCenter(u16 r, int& i, int& j) const;

		void RegionNavcellNearest(u16 r, int iGoal, int jGoal, int& iBest, int& jBest, u32& dist2Best) const;

		bool RegionNearestNavcellInGoal(u16 r, u16 i0, u16 j0, const PathGoal& goal, u16& iOut, u16& jOut, u32& dist2Best) const;

#ifdef TEST
		bool operator==(const Chunk& b) const
		{
			return (m_ChunkI == b.m_ChunkI && m_ChunkJ == b.m_ChunkJ && m_RegionsID == b.m_RegionsID && memcmp(&m_Regions, &b.m_Regions, sizeof(u16) * CHUNK_SIZE * CHUNK_SIZE) == 0);
		}
#endif
	};

	typedef std::map<RegionID, std::set<RegionID> > EdgesMap;

	void RecomputeAllEdges(pass_class_t passClass, EdgesMap& edges);
	void UpdateEdges(u8 ci, u8 cj, pass_class_t passClass, EdgesMap& edges);

	void FindReachableRegions(RegionID from, std::set<RegionID>& reachable, pass_class_t passClass);

	void FindPassableRegions(std::set<RegionID>& regions, pass_class_t passClass);

	void FindGoalRegions(u16 gi, u16 gj, const PathGoal& goal, std::set<RegionID>& regions, pass_class_t passClass);

	/*
	 * Helpers for the A* implementation of MakeGoalReachable.
	 * We reuse flat_XX containers to have good cache locality and avoid the cost of allocating memory. Flat_XX implementa  map/set as a sorted vector
	 */
	boost::container::flat_map<RegionID, HierarchicalPathfinder::RegionID> m_Astar_Predecessor;
	boost::container::flat_map<RegionID, int> m_Astar_GScore;
	boost::container::flat_map<RegionID, int> m_Astar_FScore;
	boost::container::flat_set<RegionID> m_Astar_ClosedNodes;
	boost::container::flat_set<RegionID> m_Astar_OpenNodes;

	inline int DistBetween(const RegionID& a, const RegionID& b)
	{
		return (abs(a.ci - b.ci) + abs(a.cj - b.cj)) * CHUNK_SIZE - 30;
	};

	/**
	 * Updates @p iGoal and @p jGoal to the navcell that is the nearest to the
	 * initial goal coordinates, in one of the given @p regions.
	 * (Assumes @p regions is non-empty.)
	 */
	void FindNearestNavcellInRegions(const std::set<RegionID>& regions, u16& iGoal, u16& jGoal, pass_class_t passClass);

	Chunk& GetChunk(u8 ci, u8 cj, pass_class_t passClass)
	{
		return m_Chunks[passClass].at(cj * m_ChunksW + ci);
	}

	void FillRegionOnGrid(const RegionID& region, pass_class_t passClass, u16 value, Grid<u16>& grid);

	u16 m_W, m_H;
	u16 m_ChunksW, m_ChunksH;
	std::map<pass_class_t, std::vector<Chunk> > m_Chunks;

	std::map<pass_class_t, EdgesMap> m_Edges;

	std::map<pass_class_t, std::map<RegionID, GlobalRegionID>> m_GlobalRegions;
	std::vector<GlobalRegionID> m_AvailableGlobalRegionIDs; // TODO: actually push back deleted global regions here.

	// Passability classes for which grids will be updated when calling Update
	std::map<std::string, pass_class_t> m_PassClassMasks;

	void AddDebugEdges(pass_class_t passClass);
	HierarchicalOverlay* m_DebugOverlay;
	const CSimContext* m_SimContext; // Used for drawing the debug lines

public:
	std::vector<SOverlayLine> m_DebugOverlayLines;
};

class HierarchicalOverlay : public TerrainTextureOverlay
{
public:
	HierarchicalPathfinder& m_PathfinderHier;

	HierarchicalOverlay(HierarchicalPathfinder& pathfinderHier) :
		TerrainTextureOverlay(Pathfinding::NAVCELLS_PER_TILE), m_PathfinderHier(pathfinderHier)
	{
	}

	virtual void BuildTextureRGBA(u8* data, size_t w, size_t h)
	{
		pass_class_t passClass = m_PathfinderHier.GetPassabilityClass("default");

		for (size_t j = 0; j < h; ++j)
		{
			for (size_t i = 0; i < w; ++i)
			{
				SColor4ub color;

				HierarchicalPathfinder::RegionID rid = m_PathfinderHier.Get(i, j, passClass);
				if (rid.r == 0)
					color = SColor4ub(0, 0, 0, 0);
				else if (rid.r == 0xFFFF)
					color = SColor4ub(255, 0, 255, 255);
				else
					color = GetColor(rid.r + rid.ci*5 + rid.cj*7, 127);

				*data++ = color.R;
				*data++ = color.G;
				*data++ = color.B;
				*data++ = color.A;
			}
		}
	}
};


#endif // INCLUDED_HIERPATHFINDER
