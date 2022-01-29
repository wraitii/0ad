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

#include "simulation2/system/Component.h"
#include "ICmpWaterManager.h"

#include "graphics/RenderableObject.h"
#include "graphics/Terrain.h"
#include "graphics/Water.h"
#include "renderer/Renderer.h"
#include "renderer/SceneRenderer.h"
#include "ps/World.h"
#include "simulation2/MessageTypes.h"
#include "tools/atlas/GameInterface/GameLoop.h"

class CCmpWaterManager final : public ICmpWaterManager
{
public:
	static void ClassInit(CComponentManager& componentManager)
	{
		// No need to subscribe to WaterChanged since we're actually the one sending those.
		componentManager.SubscribeToMessageType(MT_TerrainChanged);
	}

	DEFAULT_COMPONENT_ALLOCATOR(WaterManager)

	// Dynamic state:

	// May be absent.
	CWater* m_Water = nullptr;

	entity_pos_t m_WaterHeight;

	static std::string GetSchema()
	{
		return "<a:component type='system'/><empty/>";
	}

	void Init(const CParamNode& UNUSED(paramNode)) override
	{
		CWorld* world = GetSimContext().GetWorld();
		if (world)
			m_Water = world->GetWater();
	}

	void Deinit() override
	{
	}

	void Serialize(ISerializer& serialize) override
	{
		serialize.NumberFixed_Unbounded("height", m_WaterHeight);
	}

	void Deserialize(const CParamNode& paramNode, IDeserializer& deserialize) override
	{
		Init(paramNode);

		deserialize.NumberFixed_Unbounded("height", m_WaterHeight);
	}

	void HandleMessage(const CMessage& msg, bool UNUSED(global)) override
	{
		switch (msg.GetType())
		{
			case MT_TerrainChanged:
			{
				// Tell the renderer to redraw part of the map.
				if (CRenderer::IsInitialised())
				{
					const CMessageTerrainChanged& msgData = static_cast<const CMessageTerrainChanged&> (msg);
					GetSimContext().GetTerrain().MakeDirty(msgData.i0,msgData.j0,msgData.i1,msgData.j1,RENDERDATA_UPDATE_VERTICES);
				}
				break;
			}
		}
	}

	virtual void SetWaterLevel(entity_pos_t h)
	{
		if (m_WaterHeight == h)
			return;

		m_WaterHeight = h;

		if (m_Water)
			m_Water->SetWaterHeight(h.ToFloat());

		CMessageWaterChanged msg;
		GetSimContext().GetComponentManager().BroadcastMessage(msg);
	}

	entity_pos_t GetWaterLevel(entity_pos_t UNUSED(x), entity_pos_t UNUSED(z)) const override
	{
		return m_WaterHeight;
	}

	float GetExactWaterLevel(float UNUSED(x), float UNUSED(z)) const override
	{
		return m_WaterHeight.ToFloat();
	}
};

REGISTER_COMPONENT_TYPE(WaterManager)
