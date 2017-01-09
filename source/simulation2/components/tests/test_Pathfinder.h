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

#include "simulation2/system/ComponentTest.h"

#define TEST

#include "simulation2/components/ICmpObstructionManager.h"
#include "simulation2/components/ICmpPathfinder.h"
#include "simulation2/components/CCmpPathfinder_Common.h"

#include "graphics/MapReader.h"
#include "graphics/Terrain.h"
#include "graphics/TerrainTextureManager.h"
#include "lib/timer.h"
#include "lib/tex/tex.h"
#include "ps/Loader.h"
#include "ps/Pyrogenesis.h"
#include "simulation2/Simulation2.h"

class TestCmpPathfinder : public CxxTest::TestSuite
{
public:
	void setUp()
	{
		g_VFS = CreateVfs(20 * MiB);
		g_VFS->Mount(L"", DataDir()/"mods"/"mod", VFS_MOUNT_MUST_EXIST);
		g_VFS->Mount(L"", DataDir()/"mods"/"public", VFS_MOUNT_MUST_EXIST, 1); // ignore directory-not-found errors
		TS_ASSERT_OK(g_VFS->Mount(L"cache", DataDir()/"_testcache"));

		CXeromyces::Startup();

		// Need some stuff for terrain movement costs:
		// (TODO: this ought to be independent of any graphics code)
		new CTerrainTextureManager;
		g_TexMan.LoadTerrainTextures();
	}

	void tearDown()
	{
		delete &g_TexMan;
		CXeromyces::Terminate();
		g_VFS.reset();
		DeleteDirectory(DataDir()/"_testcache");
	}

	void test_namespace()
	{
		// Check that Pathfinding::NAVCELL_SIZE is actually an integer and that the definitions
		// of Pathfinding::NAVCELL_SIZE_INT and Pathfinding::NAVCELL_SIZE_LOG2 match
		TS_ASSERT_EQUALS(Pathfinding::NAVCELL_SIZE.ToInt_RoundToNegInfinity(), Pathfinding::NAVCELL_SIZE.ToInt_RoundToInfinity());
		TS_ASSERT_EQUALS(Pathfinding::NAVCELL_SIZE.ToInt_RoundToNearest(), Pathfinding::NAVCELL_SIZE_INT);
		TS_ASSERT_EQUALS((Pathfinding::NAVCELL_SIZE >> 1).ToInt_RoundToZero(), Pathfinding::NAVCELL_SIZE_LOG2);
	}

	void hierarchical_globalRegions_testmap(std::wstring map)
	{
		CTerrain terrain;

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(map,
						   sim2.GetScriptInterface().GetJSRuntime(), JS::UndefinedHandleValue,
						   &terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
						   &sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		pass_class_t obstructionsMask = cmpPathfinder->GetPassabilityClass("default");
		HierarchicalPathfinder& hier = ((CCmpPathfinder*)cmpPathfinder.operator->())->m_LongPathfinder.GetHierarchicalPathfinder();

		std::map<HierarchicalPathfinder::RegionID, HierarchicalPathfinder::GlobalRegionID> globalRegions = hier.m_GlobalRegions[obstructionsMask];

#ifdef DEBUG
		// speed things up a little for debug mode otherwise it'll take ages.
		for (u8 cj = 4; cj < hier.m_ChunksH; cj += 16)
			for (u8 ci = 4; ci < hier.m_ChunksW; ci += 16)
#else
		for (u8 cj = 0; cj < hier.m_ChunksH; cj += 2)
			for (u8 ci = 0; ci < hier.m_ChunksW; ci += 2)
#endif
				for(u16 i : hier.GetChunk(ci, cj, obstructionsMask).m_RegionsID)
				{
					std::set<HierarchicalPathfinder::RegionID> reachables;
					hier.FindReachableRegions(HierarchicalPathfinder::RegionID{ci, cj, i}, reachables, obstructionsMask);
					HierarchicalPathfinder::GlobalRegionID ID = globalRegions[HierarchicalPathfinder::RegionID{ci, cj, i}];
					for (HierarchicalPathfinder::RegionID region : reachables)
						TS_ASSERT_EQUALS(ID, globalRegions[region]);
				}
	}

	void test_hierarchical_globalRegions()
	{
		// This test validates that the hierarchical's pathfinder global regions are in accordance with its regions
		// IE it asserts that, for any two regions A and B of the hierarchical pathfinder, if one can find a path from A to B
		// then A and B have the same global region.
		std::vector<std::wstring> maps = { L"maps/scenarios/Peloponnese.pmp", L"maps/skirmishes/Corinthian Isthmus (2).pmp", L"maps/skirmishes/Greek Acropolis (2).pmp" };

		for (std::wstring t : maps)
			hierarchical_globalRegions_testmap(t);
	}

	void hierarchical_update_testmap(std::wstring map)
	{
		CTerrain terrain;

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(map,
						   sim2.GetScriptInterface().GetJSRuntime(), JS::UndefinedHandleValue,
						   &terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
						   &sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		pass_class_t obstructionsMask = cmpPathfinder->GetPassabilityClass("default");
		HierarchicalPathfinder& hier = ((CCmpPathfinder*)cmpPathfinder.operator->())->m_LongPathfinder.GetHierarchicalPathfinder();

		// make copies
		const auto pristine_GR = hier.m_GlobalRegions;
		const auto pristine_Chunks = hier.m_Chunks;

		Grid<NavcellData>* pathfinderGrid = ((CCmpPathfinder*)cmpPathfinder.operator->())->m_LongPathfinder.m_Grid;

		Grid<u8> dirtyGrid(hier.m_ChunksW * HierarchicalPathfinder::CHUNK_SIZE,hier.m_ChunksH * HierarchicalPathfinder::CHUNK_SIZE);
		srand(1234);

#ifdef DEBUG
		size_t tries = 2;
#else
		size_t tries = 20;
#endif
		for (size_t i = 0; i < tries; ++i)
		{
			// Dirty a random one
			dirtyGrid.reset();
			u8 ci = rand() % (hier.m_ChunksW-10) + 8;
			u8 cj = rand() % (hier.m_ChunksH-10) + 8;
			dirtyGrid.set(ci * HierarchicalPathfinder::CHUNK_SIZE + 4, cj * HierarchicalPathfinder::CHUNK_SIZE + 4, 1);
			hier.Update(pathfinderGrid, dirtyGrid);

			// global regions may have changed, but we validate those in another test so that's fine.
			// formally speaking we should rather validate that regions exist with the same pixels, but so far
			// re-initing regions will keep the same IDs for the same pixels so this is OK.
			TS_ASSERT_EQUALS(hier.m_Chunks.at(obstructionsMask), pristine_Chunks.at(obstructionsMask));
			
		}
	}

	void test_hierarchical_update()
	{
		// This test validates that the "Update" function of the hierarchical pathfinder
		// ends up in a correct state (by comparing it with the clean, "Recompute"-d state).
		std::vector<std::wstring> maps = { L"maps/scenarios/Peloponnese.pmp", L"maps/skirmishes/Corinthian Isthmus (2).pmp", L"maps/skirmishes/Greek Acropolis (2).pmp" };

		for (std::wstring t : maps)
			hierarchical_update_testmap(t);
	}

	void test_performance_DISABLED()
	{
		CTerrain terrain;

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(L"maps/skirmishes/Median Oasis (2).pmp",
			sim2.GetScriptInterface().GetJSRuntime(), JS::UndefinedHandleValue,
			&terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			&sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		CmpPtr<ICmpPathfinder> cmp(sim2, SYSTEM_ENTITY);

#if 0
		entity_pos_t x0 = entity_pos_t::FromInt(10);
		entity_pos_t z0 = entity_pos_t::FromInt(495);
		entity_pos_t x1 = entity_pos_t::FromInt(500);
		entity_pos_t z1 = entity_pos_t::FromInt(495);
		ICmpPathfinder::Goal goal = { ICmpPathfinder::Goal::POINT, x1, z1 };

		WaypointPath path;
		cmp->ComputePath(x0, z0, goal, cmp->GetPassabilityClass("default"), path);
		for (size_t i = 0; i < path.m_Waypoints.size(); ++i)
			printf("%d: %f %f\n", (int)i, path.m_Waypoints[i].x.ToDouble(), path.m_Waypoints[i].z.ToDouble());
#endif

		double t = timer_Time();

		srand(1234);
		for (size_t j = 0; j < 1024*2; ++j)
		{
			entity_pos_t x0 = entity_pos_t::FromInt(rand() % 512);
			entity_pos_t z0 = entity_pos_t::FromInt(rand() % 512);
			entity_pos_t x1 = x0 + entity_pos_t::FromInt(rand() % 64);
			entity_pos_t z1 = z0 + entity_pos_t::FromInt(rand() % 64);
			PathGoal goal = { PathGoal::POINT, x1, z1 };

			WaypointPath path;
			cmp->ComputePath(x0, z0, goal, cmp->GetPassabilityClass("default"), path);
		}

		t = timer_Time() - t;
		printf("[%f]", t);
	}

	void test_performance_short_DISABLED()
	{
		CTerrain terrain;
		terrain.Initialize(5, NULL);

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		const entity_pos_t range = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*12);

		CmpPtr<ICmpObstructionManager> cmpObstructionMan(sim2, SYSTEM_ENTITY);
		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		srand(0);
		for (size_t i = 0; i < 200; ++i)
		{
			fixed x = fixed::FromFloat(1.5f*range.ToFloat() * rand()/(float)RAND_MAX);
			fixed z = fixed::FromFloat(1.5f*range.ToFloat() * rand()/(float)RAND_MAX);
//			printf("# %f %f\n", x.ToFloat(), z.ToFloat());
			cmpObstructionMan->AddUnitShape(INVALID_ENTITY, x, z, fixed::FromInt(2), 0, INVALID_ENTITY);
		}

		NullObstructionFilter filter;
		PathGoal goal = { PathGoal::POINT, range, range };
		WaypointPath path;
		cmpPathfinder->ComputeShortPath(filter, range/3, range/3, fixed::FromInt(2), range, goal, 0, path);
		for (size_t i = 0; i < path.m_Waypoints.size(); ++i)
			printf("# %d: %f %f\n", (int)i, path.m_Waypoints[i].x.ToFloat(), path.m_Waypoints[i].z.ToFloat());
	}

	template<typename T>
	void DumpGrid(std::ostream& stream, const Grid<T>& grid, int mask)
	{
		for (u16 j = 0; j < grid.m_H; ++j)
		{
			for (u16 i = 0; i < grid.m_W; )
			{
				if (!(grid.get(i, j) & mask))
				{
					i++;
					continue;
				}

				u16 i0 = i;
				for (i = i0+1; ; ++i)
				{
					if (i >= grid.m_W || !(grid.get(i, j) & mask))
					{
						stream << "  <rect x='" << i0 << "' y='" << j << "' width='" << (i-i0) << "' height='1'/>\n";
						break;
					}
				}
			}
		}
	}

	void test_perf2_DISABLED()
	{
		CTerrain terrain;

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(L"maps/scenarios/Peloponnese.pmp",
			sim2.GetScriptInterface().GetJSRuntime(), JS::UndefinedHandleValue,
			&terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			&sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		std::ofstream stream(OsString("perf2.html").c_str(), std::ofstream::out | std::ofstream::trunc);

		CmpPtr<ICmpObstructionManager> cmpObstructionManager(sim2, SYSTEM_ENTITY);
		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		pass_class_t obstructionsMask = cmpPathfinder->GetPassabilityClass("default");
		const Grid<NavcellData>& obstructions = cmpPathfinder->GetPassabilityGrid();

		int scale = 1;
		stream << "<!DOCTYPE html>\n";
		stream << "<style>\n";
		stream << ".passability rect { fill: black; }\n";
		stream << ".astar-open rect { fill: rgba(255,255,0,0.75); }\n";
		stream << ".astar-closed rect { fill: rgba(255,0,0,0.75); }\n";
//		stream << ".astar-closed rect { fill: rgba(0,255,0,0.75); }\n";
		stream << ".path { stroke: rgba(0,0,255,0.75); stroke-width: 1; fill: none; }\n";
		stream << "</style>\n";
		stream << "<svg width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'>\n";
		stream << "<g transform='translate(0 " << obstructions.m_H*scale << ") scale(" << scale << " -" << scale << ")'>\n";

		stream << " <g class='passability'>\n";
		DumpGrid(stream, obstructions, obstructionsMask);
		stream << " </g>\n";

		DumpPath(stream, 128*4, 256*4, 128*4, 384*4, cmpPathfinder);
// 	  	RepeatPath(500, 128*4, 256*4, 128*4, 384*4, cmpPathfinder);
//
// 		DumpPath(stream, 128*4, 204*4, 192*4, 204*4, cmpPathfinder);
//
// 		DumpPath(stream, 128*4, 230*4, 32*4, 230*4, cmpPathfinder);

		stream << "</g>\n";
		stream << "</svg>\n";
	}

	void test_perf3_DISABLED()
	{
		CTerrain terrain;

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(L"maps/scenarios/Peloponnese.pmp",
			sim2.GetScriptInterface().GetJSRuntime(), JS::UndefinedHandleValue,
			&terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			&sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		std::ofstream stream(OsString("perf3.html").c_str(), std::ofstream::out | std::ofstream::trunc);

		CmpPtr<ICmpObstructionManager> cmpObstructionManager(sim2, SYSTEM_ENTITY);
		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		pass_class_t obstructionsMask = cmpPathfinder->GetPassabilityClass("default");
		const Grid<NavcellData>& obstructions = cmpPathfinder->GetPassabilityGrid();

		int scale = 31;
		stream << "<!DOCTYPE html>\n";
		stream << "<style>\n";
		stream << ".passability rect { fill: black; }\n";
		stream << ".astar-open rect { fill: rgba(255,255,0,0.75); }\n";
		stream << ".astar-closed rect { fill: rgba(255,0,0,0.75); }\n";
		stream << ".path { stroke: rgba(0,0,255,0.75); stroke-width: " << (1.0f / scale) << "; fill: none; }\n";
		stream << "</style>\n";
		stream << "<svg width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'>\n";
		stream << "<defs><pattern id='grid' width='1' height='1' patternUnits='userSpaceOnUse'>\n";
		stream << "<rect fill='none' stroke='#888' stroke-width='" << (1.0f / scale) << "' width='" << scale << "' height='" << scale << "'/>\n";
		stream << "</pattern></defs>\n";
		stream << "<g transform='translate(0 " << obstructions.m_H*scale << ") scale(" << scale << " -" << scale << ")'>\n";

		stream << " <g class='passability'>\n";
		DumpGrid(stream, obstructions, obstructionsMask);
		stream << " </g>\n";

		for (int j = 160; j < 190; ++j)
			for (int i = 220; i < 290; ++i)
				DumpPath(stream, 230, 178, i, j, cmpPathfinder);

		stream << "<rect fill='url(#grid)' width='100%' height='100%'/>\n";
		stream << "</g>\n";
		stream << "</svg>\n";
	}

	static const size_t scale = 1;

	void MakeGoalReachable_testIteration(CStr& map, u16 sx, u16 sz, u16 gx, u16 gz)
	{
		int colors[26][3] = {
			{ 255, 0, 0 },
			{ 0, 255, 0 },
			{ 0, 0, 255 },
			{ 255, 255, 0 },
			{ 255, 0, 255 },
			{ 0, 255, 255 },
			{ 255, 255, 255 },

			{ 127, 0, 0 },
			{ 0, 127, 0 },
			{ 0, 0, 127 },
			{ 127, 127, 0 },
			{ 127, 0, 127 },
			{ 0, 127, 127 },
			{ 127, 127, 127},

			{ 255, 127, 0 },
			{ 127, 255, 0 },
			{ 255, 0, 127 },
			{ 127, 0, 255},
			{ 0, 255, 127 },
			{ 0, 127, 255},
			{ 255, 127, 127},
			{ 127, 255, 127},
			{ 127, 127, 255},

			{ 127, 255, 255 },
			{ 255, 127, 255 },
			{ 255, 255, 127 },
		};

		// Load up a map, dump hierarchical regions
		// From a few positions test making a few positions reachable.
		// Check performance and output results as svg files so user can verify sanity.

		CTerrain terrain;

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(map.FromUTF8(),
						   sim2.GetScriptInterface().GetJSRuntime(), JS::UndefinedHandleValue,
						   &terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
						   &sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		map.Replace(".pmp",""); map.Replace("/","");
		CStr path("MGR_" + map + "_"  + CStr::FromUInt(sx) + "_"   + CStr::FromUInt(sz) + "_" + CStr::FromUInt(gx) + "_" + CStr::FromUInt(gz) + ".html");
		std::cout << path << std::endl;
		std::ofstream stream(OsString(path).c_str(), std::ofstream::out | std::ofstream::trunc);

		CmpPtr<ICmpObstructionManager> cmpObstructionManager(sim2, SYSTEM_ENTITY);
		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		pass_class_t obstructionsMask = cmpPathfinder->GetPassabilityClass("default");
		const Grid<NavcellData>& obstructions = cmpPathfinder->GetPassabilityGrid();

		// Dump as canvas. This is terrible code but who cares.
		stream << "<!DOCTYPE html>\n";
		stream << "<head>\n"
					"<style>\n"
						"canvas { position:absolute; left:0px; top: 0px; }"
						"canvas #hier { opacity:0.5; }"
						"input[type=\"checkbox\"] { height: 50px; width:50px; background:red; -webkit-appearance:none; }input[type=\"checkbox\"]:checked { background:blue; }"
						"p.pixel { z-index:20000; display:block; position:absolute; width: " << scale << "px; height: " << scale << "px;}"
					"</style>\n"
				  "</head>\n"
				  "<body>";

		stream << "<p style='z-index: 200; background: white; position: absolute;transform:scale(2); transform-origin:0 0;'>\n";
		stream << "<input type='checkbox' checked='true' id='gridcb' onchange=\"var checked = document.getElementById('gridcb').checked; document.getElementById('grid').style.display = checked ? 'block' :' none';\">Display Grid</input>";
		stream << "<input type='checkbox' checked='true' id='hiercb' onchange=\"var checked = document.getElementById('hiercb').checked; document.getElementById('hier').style.display = checked ? 'block' :' none';\">Display Hierarchical grid </input>";
		stream << "<input type='checkbox' checked='true' id='hier2cb' onchange=\"var checked = document.getElementById('hier2cb').checked; document.getElementById('hier2').style.display = checked ? 'block' :' none';\">Display Global Regions </input>";
		stream << "<input type='checkbox' checked='true' id='pathcb' onchange=\"var checked = document.getElementById('pathcb').checked; document.getElementById('path').style.display = checked ? 'block' :' none';\">Display Path search </input>";
		stream << "<input type='checkbox' checked='true' id='path2cb' onchange=\"var checked = document.getElementById('path2cb').checked; document.getElementById('path2').style.display = checked ? 'block' :' none';\">Display path lookups </input>";
		stream << "<input id='scaleinput' type='number' value='1' onchange=\"var nscale = +document.getElementById('scaleinput').value; scale=nscale; printGrid(scale);printHier(scale);\">";
		stream << "<input id='totoro' type='range' min='0' max='100' value='1' oninput=\"var nscale = +document.getElementById('totoro').value; printPath(scale, nscale);\">\n";
		stream << "</p>";

		stream << "<canvas id='grid' width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'></canvas>";

		// set up grid
		stream << "<script>"
					"var scale = " << scale << ";\n"
					"function printGrid(scale) {\n"
					"var grid = document.getElementById('grid');\n"
					"grid.width = scale * " << obstructions.m_W << ";"
					"grid.height = scale * " << obstructions.m_H << ";"
					"var context = grid.getContext('2d');"
					"context.clearRect(0, 0, grid.width, grid.height);"
					"context.fillStyle = '#000000';\n";
		for (u16 j = 0; j < obstructions.m_H; ++j)
			for (u16 i = 0; i < obstructions.m_W; i++)
				if (obstructions.get(i, j) & obstructionsMask)
				{
					u16 i1 = i;
					for (; i1 < obstructions.m_W; i1++)
						if (!(obstructions.get(i1, j) & obstructionsMask))
							break;
					stream << "context.fillRect(" << i << " * scale," << j << " * scale," << (i1 - i) << " * scale, scale);\n";
					i = i1 - 1;
				}
		stream << "};";
		stream << "printGrid(scale)";
		stream << "</script>\n";

		// Dump hierarchical regions on another one.
		stream << "<canvas id='hier' width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'></canvas>";
		stream << "<canvas id='hier2' width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'></canvas>";

		HierarchicalPathfinder& hier = ((CCmpPathfinder*)cmpPathfinder.operator->())->m_LongPathfinder.GetHierarchicalPathfinder();

		stream << "<script>"
		"function printHier(scale) {\n"
		"var hier = document.getElementById('hier');\n"
		"hier.width = scale * " << obstructions.m_W << ";"
		"hier.height = scale * " << obstructions.m_H << ";"
		"var context = hier.getContext('2d');"
		"context.clearRect(0, 0, hier.width, hier.height);"
		"context.fillStyle = '#000000';\n";
		for (u16 j = 0; j < obstructions.m_H; ++j)
			for (u16 i = 0; i < obstructions.m_W; i++)
			{
				u16 st = hier.Get(i, j, obstructionsMask).r;
				u16 ci = i / hier.CHUNK_SIZE;
				u16 cj = j / hier.CHUNK_SIZE;
				u16 i1 = i;
				for (; i1 < obstructions.m_W; ++i1)
					if (hier.Get(i1, j, obstructionsMask).r != st || i1 / hier.CHUNK_SIZE != ci)
						break;

				if (st == 0)
					stream << "context.fillStyle = 'rgba(0,0,0,1.0)';\n";
				else if (st == 0xFFFF)
					stream << "context.fillStyle = 'rgba(255,0,255,0.5)';\n";
				else
				{
					size_t c = (st + ci*5 + cj*7) % ARRAY_SIZE(colors);
					stream << "context.fillStyle = 'rgba("<<colors[c][0]<<","<<colors[c][1]<<","<<colors[c][2]<<",1.0)';\n";
				}
				stream << "context.fillRect(" << i << " * scale," << j << " * scale," << (i1 - i) << " * scale, scale);\n";
				i = i1 - 1;
			}
		stream << "};";
		stream << "printHier(scale);\n"
		"function printHier2(scale) {\n"
		"var hier2 = document.getElementById('hier2');\n"
		"hier2.width = scale * " << obstructions.m_W << ";"
		"hier2.height = scale * " << obstructions.m_H << ";"
		"var context = hier2.getContext('2d');"
		"context.clearRect(0, 0, hier2.width, hier2.height);"
		"context.fillStyle = '#000000';\n";
		for (u16 j = 0; j < obstructions.m_H; ++j)
			for (u16 i = 0; i < obstructions.m_W; i++)
			{
				u32 globalID = hier.GetGlobalRegion(i,j,obstructionsMask);
				u16 i1 = i;
				for (; i1 < obstructions.m_W; ++i1)
					if (hier.GetGlobalRegion(i1,j,obstructionsMask) != globalID)
						break;

				if (globalID == 0)
					stream << "context.fillStyle = 'rgba(0,0,0,1.0)';\n";
				else
				{
					size_t c = globalID % ARRAY_SIZE(colors);
					stream << "context.fillStyle = 'rgba("<<colors[c][0]<<","<<colors[c][1]<<","<<colors[c][2]<<",1.0)';\n";
				}
				stream << "context.fillRect(" << i << " * scale," << j << " * scale," << (i1 - i) << " * scale, scale);\n";
				i = i1 - 1;
			}
		stream << "};";
		stream << "printHier2(scale)";
		stream << "</script>\n";

		// Ok let's check out MakeGoalReachable
		// pick a point
		fixed X,Z;
		X = fixed::FromInt(sx);
		Z = fixed::FromInt(sz);
		u16 gridSize = obstructions.m_W;
		// Convert the start coordinates to tile indexes
		u16 i0, j0;
		Pathfinding::NearestNavcell(X, Z, i0, j0, gridSize, gridSize);

		// Dump as HTML so that it's on top and add fancy shadows so it's easy to see.
		stream << "<p class='pixel' style='z-index:500000; border-radius:100%;box-shadow: 0 0 0 10px green, 0 0 0 12px black; left:" << i0 * scale << "px; top:" << j0 * scale << "px;'></p>";

		hier.FindNearestPassableNavcell(i0, j0, obstructionsMask);
		stream << "<p class='pixel' style='border-radius:100%;box-shadow: 0 0 0 20px blue, 0 0 0 22px red; left:" << i0 * scale << "px; top:" << j0 * scale << "px;'></p>";

		// Make the goal reachable. This includes shortening the path if the goal is in a non-passable
		// region, transforming non-point goals to reachable point goals, etc.

		PathGoal goal;
		goal.type = PathGoal::POINT;
		goal.x = fixed::FromInt(gx);
		goal.z = fixed::FromInt(gz);
		goal.u = CFixedVector2D(fixed::FromInt(1), fixed::Zero());
		goal.v = CFixedVector2D(fixed::Zero(),fixed::FromInt(1));
		goal.hh = fixed::FromInt(0);
		goal.hw = fixed::FromInt(0);

		u16 i1, j1;
		Pathfinding::NearestNavcell(goal.x, goal.z, i1, j1, gridSize, gridSize);
		stream << "<p class='pixel' style='z-index:500000; border-radius:100%;box-shadow: 0 0 0 10px red, 0 0 0 12px white; left:" << i1 * scale << "px; top:" << j1 * scale << "px;'></p>";

		PathGoal goalCopy = goal;
		hier.MakeGoalReachable(i0, j0, goalCopy, obstructionsMask);

		Pathfinding::NearestNavcell(goalCopy.x, goalCopy.z, i1, j1, gridSize, gridSize);
		stream << "<p class='pixel' style='border-radius:100%;box-shadow: 0 0 0 20px blue, 0 0 0 25px red; left:" << i1 * scale << "px; top:" << j1 * scale << "px;'></p>";


		stream << "<canvas id='path' width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'></canvas>";
		stream << "<canvas id='path2' width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'></canvas>";

		stream << "<script>"
		"var maxStep = 0;"
		"function printPath(scale, step) {\n"
		"var path = document.getElementById('path');\n"
		"path.width = scale * " << obstructions.m_W << ";"
		"path.height = scale * " << obstructions.m_H << ";"
		"var context = path.getContext('2d');"
		"context.clearRect(0, 0, path.width, path.height);"
		"var path2 = document.getElementById('path2');\n"
		"path2.width = scale * " << obstructions.m_W << ";"
		"path2.height = scale * " << obstructions.m_H << ";"
		"var context2 = path2.getContext('2d');"
		"context2.clearRect(0, 0, path2.width, path2.height);";

		goalCopy = goal;
		hier.MakeGoalReachable_Astar(i0, j0, goalCopy, obstructionsMask);//, stream);

		stream << "};";
		stream << "printPath(scale,10000);";
		stream << "document.getElementById('totoro').max = maxStep";
		stream << "</script>\n";

		Pathfinding::NearestNavcell(goalCopy.x, goalCopy.z, i1, j1, gridSize, gridSize);
		stream << "<p class='pixel' style='border-radius:100%;box-shadow: 0 0 0 20px green, 0 0 0 25px red; left:" << i1 * scale << "px; top:" << j1 * scale << "px;'></p>";

		// Perf test. This is a little primitive, but should work well enough to give an idea of the algo.

		double t = timer_Time();

		srand(1234);
		for (size_t j = 0; j < 10000; ++j)
		{
			PathGoal oldGoal = goal;
			hier.MakeGoalReachable(i0, j0, goal, obstructionsMask);
			goal = oldGoal;
		}

		t = timer_Time() - t;
		printf("\nPoint Only  Goal old:  [%f]\n", t);

		std::ofstream ostr(OsString("out.html").c_str(), std::ofstream::out | std::ofstream::trunc);

		t = timer_Time();
		for (size_t j = 0; j < 10000; ++j)
		{
			PathGoal oldGoal = goal;
			hier.MakeGoalReachable_Astar(i0, j0, goal, obstructionsMask);//, ostr);
			goal = oldGoal;
		}

		t = timer_Time() - t;
		printf("\nPoint Only  Goal new:  [%f]\n", t);


		// Replace the point with a small circle: this prevents the flood fill from early-exiting
		// whereas A* can perform about the same.
		goal.type = PathGoal::CIRCLE;
		goal.hh = fixed::FromInt(1);
		goal.hw = fixed::FromInt(1);

		t = timer_Time();

		srand(1234);
		for (size_t j = 0; j < 10000; ++j)
		{
			PathGoal oldGoal = goal;
			hier.MakeGoalReachable(i0, j0, goal, obstructionsMask);
			goal = oldGoal;
		}

		t = timer_Time() - t;
		printf("\nSmall Circle Goal old:  [%f]\n", t);

		t = timer_Time();
		for (size_t j = 0; j < 10000; ++j)
		{
			PathGoal oldGoal = goal;
			hier.MakeGoalReachable_Astar(i0, j0, goal, obstructionsMask);//, ostr);
			goal = oldGoal;
		}

		t = timer_Time() - t;
		printf("\nSmall Circle Goal new:  [%f]\n\n\n", t);

		stream << "</body>\n";
	}

	void test_MakeGoalReachable_performance()
	{
		struct test
		{
			CStr map;
			u16 sx;
			u16 sz;
			u16 gx;
			u16 gz;
		};
		/*
		 * Compare performance on a few cases:
		 * - short path, good case for the flood fill (it finds immediately the point/circle and stops)
		 * - short path, bad case for the flood fill (it will not find the correct region right away, so it's literally about 100x slower than the former)
		 * - long path around the bend, close to worst-case for A*
		 * - Long unreachable path, but the "closest point" is reachable in almost a straight direction.
		 * - Inverse of the former (the region to fill is much smaller)
		 * - large island, A* still has a lot to fill here
		 * - straight with obstructions
		 * - straight, fewer obstructions
		 * - bad case (U shape around the start containing A*)
		 * - bad case: U shape + unreachable. We need to return something reasonably close, not in the first U
		 * - bad calse: two U shapes tripping A*
		 */
		std::vector<test> maps = {
			{ "maps/scenarios/Peloponnese.pmp", 600, 800, 800, 800 },
			{ "maps/scenarios/Peloponnese.pmp", 600, 800, 600, 900 },
			{ "maps/scenarios/Peloponnese.pmp", 600, 800, 770, 1400 },
			{ "maps/scenarios/Peloponnese.pmp", 1000, 300, 1500, 1450 },
			{ "maps/scenarios/Peloponnese.pmp", 1500, 1450, 1000, 300 },
			{ "maps/skirmishes/Corsica and Sardinia (4).pmp", 300, 1300, 1300, 300 },
			{ "maps/skirmishes/Alpine_Mountains_(3).pmp", 200, 200, 800, 800 },
			{ "maps/skirmishes/Corinthian Isthmus (2).pmp", 200, 200, 800, 800 },
			{ "maps/skirmishes/Mediterranean Cove (2).pmp", 200, 200, 800, 800 },
			{ "maps/skirmishes/Dueling Cliffs (3v3).pmp", 200, 200, 800, 800 },
			{ "maps/skirmishes/Dueling Cliffs (3v3).pmp", 350, 200, 900, 900 },
			{ "maps/skirmishes/Dueling Cliffs (3v3).pmp", 200, 200, 950, 950 },
		};

		for (auto t : maps)
		{
			MakeGoalReachable_testIteration(t.map, t.sx, t.sz, t.gx, t.gz);
		}
	}

	void DumpPath(std::ostream& stream, int i0, int j0, int i1, int j1, CmpPtr<ICmpPathfinder>& cmpPathfinder)
	{
		entity_pos_t x0 = entity_pos_t::FromInt(i0);
		entity_pos_t z0 = entity_pos_t::FromInt(j0);
		entity_pos_t x1 = entity_pos_t::FromInt(i1);
		entity_pos_t z1 = entity_pos_t::FromInt(j1);

		PathGoal goal = { PathGoal::POINT, x1, z1 };

		WaypointPath path;
		cmpPathfinder->ComputePath(x0, z0, goal, cmpPathfinder->GetPassabilityClass("default"), path);

		u32 debugSteps;
		double debugTime;
		Grid<u8> debugGrid;
		cmpPathfinder->GetDebugData(debugSteps, debugTime, debugGrid);
// 		stream << " <g style='visibility:hidden'>\n";

		stream << " <g>\n";
// 		stream << " <g class='astar-open'>\n";
// 		DumpGrid(stream, debugGrid, 1);
// 		stream << " </g>\n";
// 		stream << " <g class='astar-closed'>\n";
// 		DumpGrid(stream, debugGrid, 2);
// 		stream << " </g>\n";
// 		stream << " <g class='astar-closed'>\n";
// 		DumpGrid(stream, debugGrid, 3);
// 		stream << " </g>\n";
		stream << " </g>\n";

		stream << " <polyline";
		stream << " onmouseover='this.previousElementSibling.style.visibility=\"visible\"' onmouseout='this.previousElementSibling.style.visibility=\"hidden\"'";
		double length = 0;
		for (ssize_t i = 0; i < (ssize_t)path.m_Waypoints.size()-1; ++i)
		{
			double dx = (path.m_Waypoints[i+1].x - path.m_Waypoints[i].x).ToDouble();
			double dz = (path.m_Waypoints[i+1].z - path.m_Waypoints[i].z).ToDouble();
			length += sqrt(dx*dx + dz*dz);
		}
		stream << " title='length: " << length << "; tiles explored: " << debugSteps << "; time: " << debugTime*1000 << " msec'";
		stream << " class='path' points='";
		for (size_t i = 0; i < path.m_Waypoints.size(); ++i)
			stream << path.m_Waypoints[i].x.ToDouble()*Pathfinding::NAVCELLS_PER_TILE/TERRAIN_TILE_SIZE << "," << path.m_Waypoints[i].z.ToDouble()*Pathfinding::NAVCELLS_PER_TILE/TERRAIN_TILE_SIZE << " ";
		stream << "'/>\n";
	}

	void RepeatPath(int n, int i0, int j0, int i1, int j1, CmpPtr<ICmpPathfinder>& cmpPathfinder)
	{
		entity_pos_t x0 = entity_pos_t::FromInt(i0);
		entity_pos_t z0 = entity_pos_t::FromInt(j0);
		entity_pos_t x1 = entity_pos_t::FromInt(i1);
		entity_pos_t z1 = entity_pos_t::FromInt(j1);

		PathGoal goal = { PathGoal::POINT, x1, z1 };

		double t = timer_Time();
		for (int i = 0; i < n; ++i)
		{
			WaypointPath path;
			cmpPathfinder->ComputePath(x0, z0, goal, cmpPathfinder->GetPassabilityClass("default"), path);
		}
		t = timer_Time() - t;
		debug_printf("### RepeatPath %fms each (%fs total)\n", 1000*t / n, t);
	}

};
