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

#include "HierarchicalPathfinder.h"

#include "graphics/Overlay.h"
#include "ps/Profile.h"

// Find the root ID of a region, used by InitRegions
inline u16 RootID(u16 x, const std::vector<u16>& v)
{
	while (v[x] < x)
		x = v[x];

	return x;
}

void HierarchicalPathfinder::Chunk::InitRegions(int ci, int cj, Grid<NavcellData>* grid, pass_class_t passClass)
{
	ENSURE(ci < 256 && cj < 256); // avoid overflows
	m_ChunkI = ci;
	m_ChunkJ = cj;

	memset(m_Regions, 0, sizeof(m_Regions));

	int i0 = ci * CHUNK_SIZE;
	int j0 = cj * CHUNK_SIZE;
	int i1 = std::min(i0 + CHUNK_SIZE, (int)grid->m_W);
	int j1 = std::min(j0 + CHUNK_SIZE, (int)grid->m_H);

	// Efficiently flood-fill the m_Regions grid

	int regionID = 0;
	std::vector<u16> connect;

	u16* pCurrentID = NULL;
	u16 LeftID = 0;
	u16 DownID = 0;
	bool Checked = false; // prevent some unneccessary RootID calls

	connect.reserve(32); // TODO: What's a sensible number?
	connect.push_back(0); // connect[0] = 0

	// Start by filling the grid with 0 for blocked,
	// and regionID for unblocked
	for (int j = j0; j < j1; ++j)
	{
		for (int i = i0; i < i1; ++i)
		{
			pCurrentID = &m_Regions[j-j0][i-i0];
			if (!IS_PASSABLE(grid->get(i, j), passClass))
			{
				*pCurrentID = 0;
				continue;
			}

			if (j > j0)
				DownID = m_Regions[j-1-j0][i-i0];

			if (i == i0)
				LeftID = 0;
			else
				LeftID = m_Regions[j-j0][i-1-i0];

			if (LeftID > 0)
			{
				*pCurrentID = LeftID;
				if (*pCurrentID != DownID && DownID > 0 && !Checked)
				{
					u16 id0 = RootID(DownID, connect);
					u16 id1 = RootID(LeftID, connect);
					Checked = true; // this avoids repeatedly connecting the same IDs

					if (id0 < id1)
						connect[id1] = id0;
					else if (id0 > id1)
						connect[id0] = id1;
				}
				else if (DownID == 0)
					Checked = false;
			}
			else if (DownID > 0)
			{
				*pCurrentID = DownID;
				Checked = false;
			}
			else
			{
				// New ID
				*pCurrentID = ++regionID;
				connect.push_back(regionID);
				Checked = false;
			}
		}
	}

	// Directly point the root ID
	for (u16 i = 1; i < regionID+1; ++i)
	{
		if (connect[i] != i)
			connect[i] = RootID(i, connect);
		if (std::find(m_RegionsID.begin(),m_RegionsID.end(), connect[i]) == m_RegionsID.end())
			m_RegionsID.push_back(connect[i]);
	}

	// Replace IDs by the root ID
	for (int j = 0; j < CHUNK_SIZE; ++j)
		for (int i = 0; i < CHUNK_SIZE; ++i)
			m_Regions[j][i] = connect[m_Regions[j][i]];
}

/**
 * Returns a RegionID for the given global navcell coords
 * (which must be inside this chunk);
 */
HierarchicalPathfinder::RegionID HierarchicalPathfinder::Chunk::Get(int i, int j) const
{
	ENSURE(i < CHUNK_SIZE && j < CHUNK_SIZE);
	return RegionID(m_ChunkI, m_ChunkJ, m_Regions[j][i]);
}

/**
 * Return the global navcell coords that correspond roughly to the
 * center of the given region in this chunk.
 * (This is not guaranteed to be actually inside the region.)
 */
void HierarchicalPathfinder::Chunk::RegionCenter(u16 r, int& i_out, int& j_out) const
{
	// Find the mean of i,j coords of navcells in this region:

	u32 si = 0, sj = 0; // sum of i,j coords
	u32 n = 0; // number of navcells in region

	cassert(CHUNK_SIZE < 256); // conservative limit to ensure si and sj don't overflow

	for (int j = 0; j < CHUNK_SIZE; ++j)
	{
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			if (m_Regions[j][i] == r)
			{
				si += i;
				sj += j;
				n += 1;
			}
		}
	}

	// Avoid divide-by-zero
	if (n == 0)
		n = 1;

	i_out = m_ChunkI * CHUNK_SIZE + si / n;
	j_out = m_ChunkJ * CHUNK_SIZE + sj / n;
}

/**
 * Returns the global navcell coords, and the squared distance to the goal
 * navcell, of whichever navcell inside the given region is closest to
 * that goal.
 */
void HierarchicalPathfinder::Chunk::RegionNavcellNearest(u16 r, int iGoal, int jGoal, int& iBest, int& jBest, u32& dist2Best) const
{
	iBest = 0;
	jBest = 0;
	dist2Best = std::numeric_limits<u32>::max();

	for (int j = 0; j < CHUNK_SIZE; ++j)
	{
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			if (m_Regions[j][i] != r)
				continue;

			u32 dist2 = (i + m_ChunkI*CHUNK_SIZE - iGoal)*(i + m_ChunkI*CHUNK_SIZE - iGoal)
				        + (j + m_ChunkJ*CHUNK_SIZE - jGoal)*(j + m_ChunkJ*CHUNK_SIZE - jGoal);

			if (dist2 < dist2Best)
			{
				iBest = i + m_ChunkI*CHUNK_SIZE;
				jBest = j + m_ChunkJ*CHUNK_SIZE;
				dist2Best = dist2;
			}
		}
	}
}

/**
 * Gives the global navcell coords, and the squared distance to the (i0,j0)
 * navcell, of whichever navcell inside the given region and inside the given goal
 * is closest to (i0,j0)
 * Returns true if the goal is inside the region, false otherwise.
 */
bool HierarchicalPathfinder::Chunk::RegionNearestNavcellInGoal(u16 r, u16 i0, u16 j0, const PathGoal& goal, u16& iOut, u16& jOut, u32& dist2Best) const
{
	// TODO: this should be optimized further.
	// Most used cases empirically seem to be SQUARE, INVERTED_CIRCLE and then POINT and CIRCLE somehwat equally
	iOut = 0;
	jOut = 0;
	dist2Best = std::numeric_limits<u32>::max();

	// Calculate the navcell that contains the center of the goal.
	int gi = (goal.x >> Pathfinding::NAVCELL_SIZE_LOG2).ToInt_RoundToNegInfinity();
	int gj = (goal.z >> Pathfinding::NAVCELL_SIZE_LOG2).ToInt_RoundToNegInfinity();

	switch(goal.type)
	{
	case PathGoal::POINT:
	{
		if (gi/CHUNK_SIZE != m_ChunkI || gj/CHUNK_SIZE != m_ChunkJ)
			return false;
		if (m_Regions[gj-m_ChunkJ * CHUNK_SIZE][gi-m_ChunkI * CHUNK_SIZE] == r)
		{
			iOut = gi;
			jOut = gj;
			dist2Best = (gi - i0)*(gi - i0)
					  + (gj - j0)*(gj - j0);
			return true;
		}
		return false;
	}
	case PathGoal::CIRCLE:
	case PathGoal::SQUARE:
	{
		// restrict ourselves to a square surrounding the goal.
		int radius = (std::max(goal.hw*3/2,goal.hh*3/2) >> Pathfinding::NAVCELL_SIZE_LOG2).ToInt_RoundToInfinity();
		int imin = std::max(0, gi-m_ChunkI*CHUNK_SIZE-radius);
		int imax = std::min((int)CHUNK_SIZE, gi-m_ChunkI*CHUNK_SIZE+radius+1);
		int jmin = std::max(0, gj-m_ChunkJ*CHUNK_SIZE-radius);
		int jmax = std::min((int)CHUNK_SIZE, gj-m_ChunkJ*CHUNK_SIZE+radius+1);
		bool found = false;
		u32 dist2 = std::numeric_limits<u32>::max();
		for (u16 j = jmin; j < jmax; ++j)
		{
			for (u16 i = imin; i < imax; ++i)
			{
				if (m_Regions[j][i] != r)
					continue;

				if (found)
				{
					dist2 = (i + m_ChunkI*CHUNK_SIZE - i0)*(i + m_ChunkI*CHUNK_SIZE - i0)
						+ (j + m_ChunkJ*CHUNK_SIZE - j0)*(j + m_ChunkJ*CHUNK_SIZE - j0);
					if (dist2 >= dist2Best)
						continue;
				}

				if (goal.NavcellContainsGoal(m_ChunkI * CHUNK_SIZE + i, m_ChunkJ * CHUNK_SIZE + j))
				{
					if (!found)
					{
						found = true;
						dist2 = (i + m_ChunkI*CHUNK_SIZE - i0)*(i + m_ChunkI*CHUNK_SIZE - i0)
							+ (j + m_ChunkJ*CHUNK_SIZE - j0)*(j + m_ChunkJ*CHUNK_SIZE - j0);
					}
					iOut = i + m_ChunkI*CHUNK_SIZE;
					jOut = j + m_ChunkJ*CHUNK_SIZE;
					dist2Best = dist2;
				}
			}
		}
		return found;
	}
	case PathGoal::INVERTED_CIRCLE:
	case PathGoal::INVERTED_SQUARE:
	{
		bool found = false;
		u32 dist2 = std::numeric_limits<u32>::max();
		// loop over all navcells.
		for (u16 j = 0; j < CHUNK_SIZE; ++j)
		{
			for (u16 i = 0; i < CHUNK_SIZE; ++i)
			{
				if (m_Regions[j][i] != r)
					continue;

				if (found)
				{
					dist2 = (i + m_ChunkI*CHUNK_SIZE - i0)*(i + m_ChunkI*CHUNK_SIZE - i0)
						+ (j + m_ChunkJ*CHUNK_SIZE - j0)*(j + m_ChunkJ*CHUNK_SIZE - j0);
					if (dist2 >= dist2Best)
						continue;
				}

				if (goal.NavcellContainsGoal(m_ChunkI * CHUNK_SIZE + i, m_ChunkJ * CHUNK_SIZE + j))
				{
					if (!found)
					{
						found = true;
						dist2 = (i + m_ChunkI*CHUNK_SIZE - i0)*(i + m_ChunkI*CHUNK_SIZE - i0)
							+ (j + m_ChunkJ*CHUNK_SIZE - j0)*(j + m_ChunkJ*CHUNK_SIZE - j0);
					}
					iOut = i + m_ChunkI*CHUNK_SIZE;
					jOut = j + m_ChunkJ*CHUNK_SIZE;
					dist2Best = dist2;
				}
			}
		}
		return found;
	}
	}
	return false;
}

HierarchicalPathfinder::HierarchicalPathfinder() : m_DebugOverlay(NULL)
{
}

HierarchicalPathfinder::~HierarchicalPathfinder()
{
	SAFE_DELETE(m_DebugOverlay);
}

void HierarchicalPathfinder::SetDebugOverlay(bool enabled, const CSimContext* simContext)
{
	if (enabled && !m_DebugOverlay)
	{
		m_DebugOverlay = new HierarchicalOverlay(*this);
		m_DebugOverlayLines.clear();
		m_SimContext = simContext;
		AddDebugEdges(GetPassabilityClass("default"));
	}
	else if (!enabled && m_DebugOverlay)
	{
		SAFE_DELETE(m_DebugOverlay);
		m_DebugOverlayLines.clear();
		m_SimContext = NULL;
	}
}

void HierarchicalPathfinder::Recompute(Grid<NavcellData>* grid,
	const std::map<std::string, pass_class_t>& nonPathfindingPassClassMasks,
	const std::map<std::string, pass_class_t>& pathfindingPassClassMasks)
{
	PROFILE3("Hierarchical Recompute");

	m_PassClassMasks = pathfindingPassClassMasks;

	std::map<std::string, pass_class_t> allPassClasses = m_PassClassMasks;
	allPassClasses.insert(nonPathfindingPassClassMasks.begin(), nonPathfindingPassClassMasks.end());

	m_W = grid->m_W;
	m_H = grid->m_H;

	// Divide grid into chunks with round-to-positive-infinity
	m_ChunksW = (grid->m_W + CHUNK_SIZE - 1) / CHUNK_SIZE;
	m_ChunksH = (grid->m_H + CHUNK_SIZE - 1) / CHUNK_SIZE;

	ENSURE(m_ChunksW < 256 && m_ChunksH < 256); // else the u8 Chunk::m_ChunkI will overflow

	m_Chunks.clear();
	m_Edges.clear();

	// reset global regions
	m_AvailableGlobalRegionIDs.clear();
	m_AvailableGlobalRegionIDs.push_back(1);

	for (auto& passClassMask : allPassClasses)
	{
		pass_class_t passClass = passClassMask.second;

		// Compute the regions within each chunk
		m_Chunks[passClass].resize(m_ChunksW*m_ChunksH);
		for (int cj = 0; cj < m_ChunksH; ++cj)
		{
			for (int ci = 0; ci < m_ChunksW; ++ci)
			{
				m_Chunks[passClass].at(cj*m_ChunksW + ci).InitRegions(ci, cj, grid, passClass);
			}
		}

		// Construct the search graph over the regions
		EdgesMap& edges = m_Edges[passClass];
		RecomputeAllEdges(passClass, edges);

		// Spread global regions.
		std::map<RegionID, GlobalRegionID>& globalRegion = m_GlobalRegions[passClass];
		globalRegion.clear();
		for (u8 cj = 0; cj < m_ChunksH; ++cj)
			for (u8 ci = 0; ci < m_ChunksW; ++ci)
				for (u16 rid : GetChunk(ci, cj, passClass).m_RegionsID)
				{
					RegionID reg{ci,cj,rid};
					if (globalRegion.find(reg) == globalRegion.end())
					{
						GlobalRegionID ID = m_AvailableGlobalRegionIDs.back();
						m_AvailableGlobalRegionIDs.pop_back();
						if (m_AvailableGlobalRegionIDs.empty())
							m_AvailableGlobalRegionIDs.push_back(ID+1);

						globalRegion.insert({ reg, ID });
						std::set<RegionID> reachable;
						FindReachableRegions(reg, reachable, passClass);
						for (const RegionID& region : reachable)
							globalRegion.insert({ region, ID });
					}
				}
	}

	if (m_DebugOverlay)
	{
		PROFILE("debug overlay");
		m_DebugOverlayLines.clear();
		AddDebugEdges(GetPassabilityClass("default"));
	}
}

void HierarchicalPathfinder::Update(Grid<NavcellData>* grid, const Grid<u8>& dirtinessGrid)
{
	PROFILE3("Hierarchical Update");

	std::vector<RegionID> updated;
	for (u8 cj = 0; cj <  m_ChunksH; ++cj)
	{
		for (u8 ci = 0; ci < m_ChunksW; ++ci)
		{
			if (!IsChunkDirty(ci, cj, dirtinessGrid))
				continue;
			for (const std::pair<std::string, pass_class_t>& passClassMask : m_PassClassMasks)
			{
				pass_class_t passClass = passClassMask.second;
				Chunk& a = m_Chunks[passClass].at(ci + cj*m_ChunksW);

				// Clean up edges and global region ID
				EdgesMap& edgeMap = m_Edges[passClass];
				for (u16 i : a.m_RegionsID)
				{
					RegionID reg{ci, cj, i};
					m_GlobalRegions[passClass].erase(reg);
					for (const RegionID& neighbor : edgeMap[reg])
					{
						edgeMap[neighbor].erase(reg);
						if (edgeMap[neighbor].empty())
							edgeMap.erase(neighbor);
					}
					edgeMap.erase(reg);
				}

				// recompute
				a.InitRegions(ci, cj, grid, passClass);

				for (u16 i : a.m_RegionsID)
					updated.push_back(RegionID{ci, cj, i});

				// add back edges
				UpdateEdges(ci, cj, passClass, edgeMap);
			}
		}
	}

	// Add back global region ID
	// To try and be clever we'll run a custom flood-fill that stops as soon as it runs into something we know,
	// and if nothing then we'll create a new global region.
	// It also keeps track of all connected regions with no IDs in case of contiguous dirtiness (likely) to be faster if possible.
	// This probably scales poorly with a large enough update?
	for (const RegionID& reg : updated)
		for (const std::pair<std::string, pass_class_t>& passClassMask : m_PassClassMasks)
		{
			std::set<RegionID> visited;
			std::vector<RegionID> open;
			std::vector<RegionID> updating = { reg };
			open.push_back(reg);

			GlobalRegionID ID = 0;
			std::map<RegionID, GlobalRegionID>& globalRegion = m_GlobalRegions[passClassMask.second];
			EdgesMap& edgeMap = m_Edges[passClassMask.second];
			while (!open.empty() && ID == 0)
			{
				RegionID curr = open.back();
				open.pop_back();
				for (const RegionID& region : edgeMap[curr])
					if (visited.insert(region).second)
					{
						open.push_back(region);
						if (globalRegion.find(region) != globalRegion.end())
						{
							ID = globalRegion.at(region);
							break;
						}
						else
							updating.push_back(region);
					}
			}
			if (ID == 0)
			{
				ID = m_AvailableGlobalRegionIDs.back();
				m_AvailableGlobalRegionIDs.pop_back();
				if (m_AvailableGlobalRegionIDs.empty())
					m_AvailableGlobalRegionIDs.push_back(ID+1);
			}
			for (const RegionID& reg : updating)
				globalRegion[reg] = ID;
	}

	if (m_DebugOverlay)
	{
		PROFILE("debug overlay");
		m_DebugOverlayLines.clear();
		AddDebugEdges(GetPassabilityClass("default"));
	}
}

bool HierarchicalPathfinder::IsChunkDirty(int ci, int cj, const Grid<u8>& dirtinessGrid) const
{
	int i0 = ci * CHUNK_SIZE;
	int j0 = cj * CHUNK_SIZE;
	int i1 = std::min(i0 + CHUNK_SIZE, (int)dirtinessGrid.m_W);
	int j1 = std::min(j0 + CHUNK_SIZE, (int)dirtinessGrid.m_H);
	for (int j = j0; j < j1; ++j)
	{
		for (int i = i0; i < i1; ++i)
		{
			if (dirtinessGrid.get(i, j))
				return true;
		}
	}
	return false;
}

/**
 * Connect a chunk's regions to their neighbors. Not optimised for global recomputing.
 * TODO: reduce code duplication with below
 */
void HierarchicalPathfinder::UpdateEdges(u8 ci, u8 cj, pass_class_t passClass, EdgesMap& edges)
{
	std::vector<Chunk>& chunks = m_Chunks[passClass];

	Chunk& a = chunks.at(cj*m_ChunksW + ci);

	if (ci > 0)
	{
		Chunk& b = chunks.at(cj*m_ChunksW + (ci-1));
		RegionID raPrev(0,0,0);
		RegionID rbPrev(0,0,0);
		for (int j = 0; j < CHUNK_SIZE; ++j)
		{
			RegionID ra = a.Get(0, j);
			RegionID rb = b.Get(CHUNK_SIZE-1, j);
			if (ra.r && rb.r)
			{
				if (ra == raPrev && rb == rbPrev)
					continue;
				edges[ra].insert(rb);
				edges[rb].insert(ra);
				raPrev = ra;
				rbPrev = rb;
			}
		}
	}

	if (ci < m_ChunksW-1)
	{
		Chunk& b = chunks.at(cj*m_ChunksW + (ci+1));
		RegionID raPrev(0,0,0);
		RegionID rbPrev(0,0,0);
		for (int j = 0; j < CHUNK_SIZE; ++j)
		{
			RegionID ra = a.Get(CHUNK_SIZE-1, j);
			RegionID rb = b.Get(0, j);
			if (ra.r && rb.r)
			{
				if (ra == raPrev && rb == rbPrev)
					continue;
				edges[ra].insert(rb);
				edges[rb].insert(ra);
				raPrev = ra;
				rbPrev = rb;
			}
		}
	}

	if (cj > 0)
	{
		Chunk& b = chunks.at((cj-1)*m_ChunksW + ci);
		RegionID raPrev(0,0,0);
		RegionID rbPrev(0,0,0);
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			RegionID ra = a.Get(i, 0);
			RegionID rb = b.Get(i, CHUNK_SIZE-1);
			if (ra.r && rb.r)
			{
				if (ra == raPrev && rb == rbPrev)
					continue;
				edges[ra].insert(rb);
				edges[rb].insert(ra);
				raPrev = ra;
				rbPrev = rb;
			}
		}
	}

	if (cj < m_ChunksH - 1)
	{
		Chunk& b = chunks.at((cj+1)*m_ChunksW + ci);
		RegionID raPrev(0,0,0);
		RegionID rbPrev(0,0,0);
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			RegionID ra = a.Get(i, CHUNK_SIZE-1);
			RegionID rb = b.Get(i, 0);
			if (ra.r && rb.r)
			{
				if (ra == raPrev && rb == rbPrev)
					continue;
				edges[ra].insert(rb);
				edges[rb].insert(ra);
				raPrev = ra;
				rbPrev = rb;
			}
		}
	}
}

/**
 * Find edges between regions in all chunks, in an optimised manner (only look at top/left)
 */
void HierarchicalPathfinder::RecomputeAllEdges(pass_class_t passClass, EdgesMap& edges)
{
	std::vector<Chunk>& chunks = m_Chunks[passClass];

	edges.clear();

	for (int cj = 0; cj < m_ChunksH; ++cj)
	{
		for (int ci = 0; ci < m_ChunksW; ++ci)
		{
			Chunk& a = chunks.at(cj*m_ChunksW + ci);

			// For each edge between chunks, we loop over every adjacent pair of
			// navcells in the two chunks. If they are both in valid regions
			// (i.e. are passable navcells) then add a graph edge between those regions.
			// (We don't need to test for duplicates since EdgesMap already uses a
			// std::set which will drop duplicate entries.)
			// But as set.insert can be quite slow on large collection, and that we usually
			// try to insert the same values, we cache the previous one for a fast test.

			if (ci > 0)
			{
				Chunk& b = chunks.at(cj*m_ChunksW + (ci-1));
				RegionID raPrev(0,0,0);
				RegionID rbPrev(0,0,0);
				for (int j = 0; j < CHUNK_SIZE; ++j)
				{
					RegionID ra = a.Get(0, j);
					RegionID rb = b.Get(CHUNK_SIZE-1, j);
					if (ra.r && rb.r)
					{
						if (ra == raPrev && rb == rbPrev)
							continue;
						edges[ra].insert(rb);
						edges[rb].insert(ra);
						raPrev = ra;
						rbPrev = rb;
					}
				}
			}

			if (cj > 0)
			{
				Chunk& b = chunks.at((cj-1)*m_ChunksW + ci);
				RegionID raPrev(0,0,0);
				RegionID rbPrev(0,0,0);
				for (int i = 0; i < CHUNK_SIZE; ++i)
				{
					RegionID ra = a.Get(i, 0);
					RegionID rb = b.Get(i, CHUNK_SIZE-1);
					if (ra.r && rb.r)
					{
						if (ra == raPrev && rb == rbPrev)
							continue;
						edges[ra].insert(rb);
						edges[rb].insert(ra);
						raPrev = ra;
						rbPrev = rb;
					}
				}
			}
		}
	}
}

/**
 * Debug visualisation of graph edges between regions.
 */
void HierarchicalPathfinder::AddDebugEdges(pass_class_t passClass)
{
	const EdgesMap& edges = m_Edges[passClass];
	const std::vector<Chunk>& chunks = m_Chunks[passClass];

	for (auto& edge : edges)
	{
		for (const RegionID& region: edge.second)
		{
			// Draw a line between the two regions' centers

			int i0, j0, i1, j1;
			chunks[edge.first.cj * m_ChunksW + edge.first.ci].RegionCenter(edge.first.r, i0, j0);
			chunks[region.cj * m_ChunksW + region.ci].RegionCenter(region.r, i1, j1);

			CFixedVector2D a, b;
			Pathfinding::NavcellCenter(i0, j0, a.X, a.Y);
			Pathfinding::NavcellCenter(i1, j1, b.X, b.Y);

			// Push the endpoints inwards a little to avoid overlaps
			CFixedVector2D d = b - a;
			d.Normalize(entity_pos_t::FromInt(1));
			a += d;
			b -= d;

			std::vector<float> xz;
			xz.push_back(a.X.ToFloat());
			xz.push_back(a.Y.ToFloat());
			xz.push_back(b.X.ToFloat());
			xz.push_back(b.Y.ToFloat());

			m_DebugOverlayLines.emplace_back();
			m_DebugOverlayLines.back().m_Color = CColor(1.0, 0.0, 0.0, 1.0);
			SimRender::ConstructLineOnGround(*m_SimContext, xz, m_DebugOverlayLines.back(), true);
		}
	}
}

HierarchicalPathfinder::RegionID HierarchicalPathfinder::Get(u16 i, u16 j, pass_class_t passClass)
{
	int ci = i / CHUNK_SIZE;
	int cj = j / CHUNK_SIZE;
	ENSURE(ci < m_ChunksW && cj < m_ChunksH);
	return m_Chunks[passClass][cj*m_ChunksW + ci].Get(i % CHUNK_SIZE, j % CHUNK_SIZE);
}

HierarchicalPathfinder::GlobalRegionID HierarchicalPathfinder::GetGlobalRegion(u16 i, u16 j, pass_class_t passClass)
{
	RegionID region = Get(i, j, passClass);
	if (region.r == 0)
		return (GlobalRegionID)0;
	return m_GlobalRegions[passClass][region];
}

void HierarchicalPathfinder::MakeGoalReachable(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass)
{
	PROFILE2("MakeGoalReachable");
	RegionID source = Get(i0, j0, passClass);

	// Find everywhere that's reachable
	std::set<RegionID> reachableRegions;

	// Flood-fill the region graph, starting at 'from',
	// collecting all the regions that are reachable via edges
	std::vector<RegionID> open;
	open.reserve(64);
	open.push_back(source);
	reachableRegions.insert(source);


	u16 bestI = 0;
	u16 bestJ = 0;
	u32 dist2Best = std::numeric_limits<u32>::max();

	u16 iGoal, jGoal;
	Pathfinding::NearestNavcell(goal.x, goal.z, iGoal, jGoal, m_W, m_H);

	std::set<RegionID> possRegs;
	FindGoalRegions(iGoal, jGoal, goal, possRegs, passClass);

	EdgesMap& edgeMap = m_Edges[passClass];
	while (!open.empty())
	{
		RegionID curr = open.back();
		open.pop_back();

		for (const RegionID& region : edgeMap[curr])
			// Add to the reachable set; if this is the first time we added
			// it then also add it to the open list
			if (reachableRegions.insert(region).second)
				open.push_back(region);

		// Skip region if its chunk doesn't contain the goal area
		entity_pos_t x0 = Pathfinding::NAVCELL_SIZE * (curr.ci * CHUNK_SIZE);
		entity_pos_t z0 = Pathfinding::NAVCELL_SIZE * (curr.cj * CHUNK_SIZE);
		entity_pos_t x1 = x0 + Pathfinding::NAVCELL_SIZE * CHUNK_SIZE;
		entity_pos_t z1 = z0 + Pathfinding::NAVCELL_SIZE * CHUNK_SIZE;
		if (!goal.RectContainsGoal(x0, z0, x1, z1))
			continue;

		u16 i,j;
		u32 dist2;

		// If the region contains the goal area, the goal is reachable
		// Remember the best point for optimization.
		if (GetChunk(curr.ci, curr.cj, passClass).RegionNearestNavcellInGoal(curr.r, i0, j0, goal, i, j, dist2))
		{
			// If it's a point, no need to move it, we're done
			if (goal.type == PathGoal::POINT)
				return;
			if (dist2 < dist2Best)
			{
				bestI = i;
				bestJ = j;
				dist2Best = dist2;
			}
			possRegs.erase(curr);
			// We ran out of potential regions so we're done
			if (possRegs.empty())
				return;
		}

	}

	// If the goal area wasn't reachable,
	// find the navcell that's nearest to the goal's center
	if (dist2Best == std::numeric_limits<u32>::max())
	{
		FindNearestNavcellInRegions(reachableRegions, iGoal, jGoal, passClass);

		// Construct a new point goal at the nearest reachable navcell
		PathGoal newGoal;
		newGoal.type = PathGoal::POINT;
		Pathfinding::NavcellCenter(iGoal, jGoal, newGoal.x, newGoal.z);
		goal = newGoal;
		return;
	}

	ENSURE(dist2Best != std::numeric_limits<u32>::max());
	PathGoal newGoal;
	newGoal.type = PathGoal::POINT;
	Pathfinding::NavcellCenter(bestI, bestJ, newGoal.x, newGoal.z);
	goal = newGoal;
}

#define OUTPUT 0

#if OUTPUT
	#include <fstream>
#endif

#if OUTPUT
// also change the header
void HierarchicalPathfinder::MakeGoalReachable_Astar(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass, std::ofstream& stream)
#else
void HierarchicalPathfinder::MakeGoalReachable_Astar(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass)
#endif
{
	/*
	 * Relatively straightforward implementation of A* on the hierarchical pathfinder graph.
	 * Since this isn't a grid, we cannot use JPS (though I'm fairly sure it could sort of be extended to work, but it's probably not trivial/worth it)
	 * Uses flat_set and flat_map over std::set and std::map since testing proves that reusing the memory ends up being more efficient
	 * The main optimisations are:
	 *	- picking the best item directly from the open list when we can be sure we know which one it is (see fasttrack)
	 *	- checking whether the goal is reachable or not, and if it isn't stopping early to avoid slowly flood-filling everything
	 * The path isn't used for now but can reconstructed using predecessor.
	 * NB: strictly speaking, from a reachability POV, we can skip A* entirely if the goal has the same global region ID (see below).
	 * Haven't done so yet and having the path may be interesting but it this function ends up being too slow it would be an idea.
	 */
	RegionID source = Get(i0, j0, passClass);

	u16 gi, gj;
	Pathfinding::NearestNavcell(goal.x, goal.z, gi, gj, m_W, m_H);

	// determine if we will be able to reach the goal.
	// If not, we can stop A* earlier by being clever.
	std::set<RegionID> goalRegions;
	FindGoalRegions(gi, gj, goal, goalRegions, passClass);

	GlobalRegionID startID = GetGlobalRegion(i0, j0, passClass);
	bool reachable = false;
	for (const RegionID& r : goalRegions)
		if (m_GlobalRegions[passClass][r] == startID)
		{
			reachable = true;
			break;
		}

#if OUTPUT
	stream << "context.fillStyle = 'rgba(1,0,1,1)';\n";
	for (const RegionID& r : goalRegions)
	{
		entity_pos_t x0 = Pathfinding::NAVCELL_SIZE * (r.ci * CHUNK_SIZE);
		entity_pos_t z0 = Pathfinding::NAVCELL_SIZE * (r.cj * CHUNK_SIZE);
		stream << "context2.fillRect(" << x0.ToInt_RoundToZero() << " * scale," << z0.ToInt_RoundToZero() << " * scale," << (int)CHUNK_SIZE << " * scale," << (int)CHUNK_SIZE << " * scale);\n";
	}
#endif

	// In general, our maps are relatively open, so it's usually a better bet to be biaised towards minimal distance over path length.
	int (*DistEstimate)(const RegionID&, u16, u16) = [](const RegionID& source, u16 gi, u16 gj) -> int { return (source.ci * CHUNK_SIZE + CHUNK_SIZE/2 - gi)*(source.ci * CHUNK_SIZE + CHUNK_SIZE/2 - gi) + (source.cj * CHUNK_SIZE + CHUNK_SIZE/2 - gj)*(source.cj * CHUNK_SIZE + CHUNK_SIZE/2 - gj); };
	// However, run unbiaised euclidian if we know the goal is unreachable, since we want to get as close as possible efficiently.
	if (!reachable)
		DistEstimate = [](const RegionID& source, u16 gi, u16 gj) -> int {
			return isqrt64((source.ci * CHUNK_SIZE + CHUNK_SIZE/2 - gi)*(source.ci * CHUNK_SIZE + CHUNK_SIZE/2 - gi) + (source.cj * CHUNK_SIZE + CHUNK_SIZE/2 - gj)*(source.cj * CHUNK_SIZE + CHUNK_SIZE/2 - gj));
		};

	m_Astar_ClosedNodes.clear();
	m_Astar_OpenNodes.clear();
	m_Astar_OpenNodes.insert(source);

	m_Astar_Predecessor.clear();

	m_Astar_GScore.clear();
	m_Astar_GScore[source] = 0;

	m_Astar_FScore.clear();
	m_Astar_FScore[source] = DistEstimate(source, gi, gj);

	RegionID current {0,0,0};

	u16 bestI, bestJ;
	u32 dist2;

	u32 timeSinceLastFScoreImprovement = 0;
#if OUTPUT
	int step = 0;
#endif

	RegionID fastTrack = source;
	int currentFScore = m_Astar_FScore[source];
	int secondBestFScore = currentFScore;
	int globalBestFScore = currentFScore;

	EdgesMap& edgeMap = m_Edges[passClass];
	while (!m_Astar_OpenNodes.empty())
	{
		// Since we are not using a fancy open list, we have to go through all nodes each time
		// So when we are sure that we know the best node (because the last run gave us a node better than us, which was already the best
		// we can fast-track and not sort but just pick that one instead.
		// In cases where the obvious path is the best, we hardly ever sort and it's a little faster
		if (fastTrack.r)
		{
			current = fastTrack;
			currentFScore = m_Astar_FScore[current];
			secondBestFScore = currentFScore;
		}
		else
		{
			auto iter = m_Astar_OpenNodes.begin();
			current = *iter;
			currentFScore = m_Astar_FScore[current];
			secondBestFScore = currentFScore;
			while (++iter != m_Astar_OpenNodes.end())
			{
				int score = m_Astar_FScore[*iter];
				if (score < currentFScore)
				{
					current = *iter;
					secondBestFScore = currentFScore;
					currentFScore = score;
				}
			}
		}

		// Stop heuristic in case we know we cannot reach the goal.
		// Indeed this would cause A* to become an inacceptably slow flood fill.
		// We keep track of our best fScore, we'll early-exit if we're too far from it
		// or we haven't found a better path in a while.
		// This will cause us to return largely suboptimal paths now and then,
		// but then again those should be rare and the player can just re-order a move.
		if (!reachable)
		{
			if (currentFScore < globalBestFScore)
			{
				globalBestFScore = currentFScore;
				timeSinceLastFScoreImprovement = 0;
			}
			else if (currentFScore > globalBestFScore * 2 || ++timeSinceLastFScoreImprovement > m_ChunksW)
				break;
		}

		entity_pos_t x0 = Pathfinding::NAVCELL_SIZE * (current.ci * CHUNK_SIZE);
		entity_pos_t z0 = Pathfinding::NAVCELL_SIZE * (current.cj * CHUNK_SIZE);
		entity_pos_t x1 = x0 + Pathfinding::NAVCELL_SIZE * CHUNK_SIZE;
		entity_pos_t z1 = z0 + Pathfinding::NAVCELL_SIZE * CHUNK_SIZE;

#if OUTPUT
		stream << "context.fillStyle = 'rgba(" <<DistEstimate(current, gi, gj) / 10000 << ",255,"<< (fastTrack.r > 0 ? 255 : 0) <<",0.8)';\n maxStep = " << step+1 << ";\n";
		stream << "if (step >= " << step << ") context.fillRect(" << x0.ToInt_RoundToZero() << " * scale," << z0.ToInt_RoundToZero() << " * scale," << (int)CHUNK_SIZE << " * scale," << (int)CHUNK_SIZE << " * scale);\n";
#endif

		fastTrack = RegionID{0,0,0};

		// TODO: we should get those first and then validate here, instead of recomputing for each.
		if (goal.RectContainsGoal(x0, z0, x1, z1))
			if (GetChunk(current.ci, current.cj, passClass).RegionNearestNavcellInGoal(current.r, i0, j0, goal, bestI, bestJ, dist2))
				break;

		m_Astar_OpenNodes.erase(current);
		m_Astar_ClosedNodes.emplace(current);
		if (reachable)
			m_Astar_FScore.erase(current);
		m_Astar_GScore.erase(current);

		int currScore = m_Astar_GScore[current];
		for (const RegionID& neighbor : edgeMap[current])
		{
			if (m_Astar_ClosedNodes.find(neighbor) != m_Astar_ClosedNodes.end())
				continue;
			int temp_m_Astar_GScore = currScore + DistBetween(neighbor, current);
			auto iter = m_Astar_OpenNodes.emplace(neighbor);
			if (!iter.second && temp_m_Astar_GScore >= m_Astar_GScore[neighbor])
				continue;
#if OUTPUT
			x0 = Pathfinding::NAVCELL_SIZE * (neighbor.ci * CHUNK_SIZE);
			z0 = Pathfinding::NAVCELL_SIZE * (neighbor.cj * CHUNK_SIZE);
			stream << "context2.fillStyle = 'rgba(255,255,0,0.3)';\n";
			stream << "if (step >= " << step << ") context2.fillRect(" << x0.ToInt_RoundToZero() << " * scale," << z0.ToInt_RoundToZero() << " * scale," << (int)CHUNK_SIZE << " * scale," << (int)CHUNK_SIZE << " * scale);\n";
#endif

			m_Astar_GScore[neighbor] = temp_m_Astar_GScore;
			// no default constructor so we'll use this hack in the meantime
			auto alreadyThere = m_Astar_Predecessor.emplace( boost::container::flat_map<RegionID, RegionID>::value_type{ neighbor, current });
			alreadyThere.first->second = current;
			int score = temp_m_Astar_GScore + DistEstimate(neighbor, gi, gj);
			if (score < secondBestFScore)
			{
				secondBestFScore = score;
				fastTrack = neighbor;
			}
			m_Astar_FScore[neighbor] = score;
		}
#if OUTPUT
		step++;
#endif
	}

	// TODO: arrive from predecessor, not origin.
	bool exactFound = GetChunk(current.ci, current.cj, passClass).RegionNearestNavcellInGoal(current.r, i0, j0, goal, bestI, bestJ, dist2);

#if OUTPUT
	fixed x0 = Pathfinding::NAVCELL_SIZE * (current.ci * CHUNK_SIZE);
	fixed z0 = Pathfinding::NAVCELL_SIZE * (current.cj * CHUNK_SIZE);
	stream << "context.fillStyle = 'rgba(255,0,0,0.6)';\n";
	stream << "if (step >= " << step << ") context.fillRect(" << x0.ToInt_RoundToZero() << " * scale," << z0.ToInt_RoundToZero() << " * scale," << (int)CHUNK_SIZE << " * scale," << (int)CHUNK_SIZE << " * scale);\n";

	// sanity check
	if (reachable)
		ENSURE (exactFound);
	else
		ENSURE (!exactFound);
#endif

	if (!exactFound)
	{
		// Pick best and roll with that.
		current = *std::min_element(m_Astar_ClosedNodes.begin(), m_Astar_ClosedNodes.end(),
									[this](const RegionID& a, const RegionID& b) -> bool { return m_Astar_FScore[a] < m_Astar_FScore[b]; });

		std::set<RegionID> set = { current };
		Pathfinding::NearestNavcell(goal.x, goal.z, bestI, bestJ, m_W, m_H);

		FindNearestNavcellInRegions(set, bestI, bestJ, passClass);
	}

	PathGoal newGoal;
	newGoal.type = PathGoal::POINT;
	Pathfinding::NavcellCenter(bestI, bestJ, newGoal.x, newGoal.z);
	goal = newGoal;
}


void HierarchicalPathfinder::FindNearestPassableNavcell(u16& i, u16& j, pass_class_t passClass)
{
	std::set<RegionID> regions;
	FindPassableRegions(regions, passClass);
	FindNearestNavcellInRegions(regions, i, j, passClass);
}

void HierarchicalPathfinder::FindNearestNavcellInRegions(const std::set<RegionID>& regions, u16& iGoal, u16& jGoal, pass_class_t passClass)
{
	// Find the navcell in the given regions that's nearest to the goal navcell:
	// * For each region, record the (squared) minimal distance to the goal point
	// * Sort regions by that underestimated distance
	// * For each region, find the actual nearest navcell
	// * Stop when the underestimated distances are worse than the best real distance

	std::vector<std::pair<u32, RegionID> > regionDistEsts; // pair of (distance^2, region)

	for (const RegionID& region : regions)
	{
		int i0 = region.ci * CHUNK_SIZE;
		int j0 = region.cj * CHUNK_SIZE;
		int i1 = i0 + CHUNK_SIZE - 1;
		int j1 = j0 + CHUNK_SIZE - 1;

		// Pick the point in the chunk nearest the goal
		int iNear = Clamp((int)iGoal, i0, i1);
		int jNear = Clamp((int)jGoal, j0, j1);

		int dist2 = (iNear - iGoal)*(iNear - iGoal)
		          + (jNear - jGoal)*(jNear - jGoal);

		regionDistEsts.emplace_back(dist2, region);
	}

	// Sort by increasing distance (tie-break on RegionID)
	std::sort(regionDistEsts.begin(), regionDistEsts.end());

	int iBest = iGoal;
	int jBest = jGoal;
	u32 dist2Best = std::numeric_limits<u32>::max();

	for (auto& pair : regionDistEsts)
	{
		if (pair.first >= dist2Best)
			break;

		RegionID region = pair.second;

		int i, j;
		u32 dist2;
		GetChunk(region.ci, region.cj, passClass).RegionNavcellNearest(region.r, iGoal, jGoal, i, j, dist2);

		if (dist2 < dist2Best)
		{
			iBest = i;
			jBest = j;
			dist2Best = dist2;
		}
	}

	iGoal = iBest;
	jGoal = jBest;
}

void HierarchicalPathfinder::FindReachableRegions(RegionID from, std::set<RegionID>& reachable, pass_class_t passClass)
{
	// Flood-fill the region graph, starting at 'from',
	// collecting all the regions that are reachable via edges

	std::vector<RegionID> open;
	open.reserve(64);
	open.push_back(from);
	reachable.insert(from);

	EdgesMap& edgeMap = m_Edges[passClass];
	while (!open.empty())
	{
		RegionID curr = open.back();
		open.pop_back();
		for (const RegionID& region : edgeMap[curr])
			// Add to the reachable set; if this is the first time we added
			// it then also add it to the open list
			if (reachable.insert(region).second)
				open.push_back(region);
	}
}

void HierarchicalPathfinder::FindPassableRegions(std::set<RegionID>& regions, pass_class_t passClass)
{
	// Construct a set of all regions of all chunks for this pass class
	for (const Chunk& chunk : m_Chunks[passClass])
	{
		for (u16 r : chunk.m_RegionsID)
			regions.insert(RegionID(chunk.m_ChunkI, chunk.m_ChunkJ, r));
	}
}

void HierarchicalPathfinder::FindGoalRegions(u16 gi, u16 gj, const PathGoal& goal, std::set<HierarchicalPathfinder::RegionID>& regions, pass_class_t passClass)
{
	if (goal.type == PathGoal::POINT)
	{
		regions.insert(Get(gi, gj, passClass));
		return;
	}

	// Find bounds
	int size = (std::max(goal.hh, goal.hw) * 3 / 2).ToInt_RoundToInfinity();

	u16 a,b; u32 c; // unused params

	for (u8 sz = (gj - size) / CHUNK_SIZE; sz <= (gj + size) / CHUNK_SIZE; ++sz)
		for (u8 sx = (gi - size) / CHUNK_SIZE; sx <= (gi + size) / CHUNK_SIZE; ++sx)
		{
			Chunk& chunk = GetChunk(sx, sz, passClass);
			for (u16 i : chunk.m_RegionsID)
				if (chunk.RegionNearestNavcellInGoal(i, 0, 0, goal, a, b, c))
					regions.insert(RegionID{sx, sz, i});
		}
}

void HierarchicalPathfinder::FillRegionOnGrid(const RegionID& region, pass_class_t passClass, u16 value, Grid<u16>& grid)
{
	ENSURE(grid.m_W == m_W && grid.m_H == m_H);

	int i0 = region.ci * CHUNK_SIZE;
	int j0 = region.cj * CHUNK_SIZE;

	const Chunk& c = m_Chunks[passClass][region.cj * m_ChunksW + region.ci];

	for (int j = 0; j < CHUNK_SIZE; ++j)
		for (int i = 0; i < CHUNK_SIZE; ++i)
			if (c.m_Regions[j][i] == region.r)
				grid.set(i0 + i, j0 + j, value);
}

Grid<u16> HierarchicalPathfinder::GetConnectivityGrid(pass_class_t passClass)
{
	Grid<u16> connectivityGrid(m_W, m_H);
	connectivityGrid.reset();

	u16 idx = 1;

	for (size_t i = 0; i < m_W; ++i)
	{
		for (size_t j = 0; j < m_H; ++j)
		{
			if (connectivityGrid.get(i, j) != 0)
				continue;

			RegionID from = Get(i, j, passClass);
			if (from.r == 0)
				continue;

			std::set<RegionID> reachable;
			FindReachableRegions(from, reachable, passClass);

			for (const RegionID& region : reachable)
				FillRegionOnGrid(region, passClass, idx, connectivityGrid);

			++idx;
		}
	}

	return connectivityGrid;
}
