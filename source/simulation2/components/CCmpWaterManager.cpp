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

#include "simulation2/system/Component.h"
#include "ICmpWaterManager.h"

#include "graphics/GameView.h"
#include "graphics/RenderableObject.h"
#include "graphics/Terrain.h"
#include "graphics/WaterManager.h"
#include "ps/Game.h"
#include "renderer/Renderer.h"
#include "simulation2/MessageTypes.h"
#include "simulation2/components/ICmpTerrain.h"

#include "tools/atlas/GameInterface/GameLoop.h"

class CCmpWaterManager : public ICmpWaterManager
{
public:
	static void ClassInit(CComponentManager& componentManager)
	{
		// No need to subscribe to WaterChanged since we're actually the one sending those.
		componentManager.SubscribeToMessageType(MT_TerrainChanged);
	}

	DEFAULT_COMPONENT_ALLOCATOR(WaterManager)

	// Dynamic state:

	entity_pos_t m_WaterHeight;

	static std::string GetSchema()
	{
		return "<a:component type='system'/><empty/>";
	}

	virtual void Init(const CParamNode& UNUSED(paramNode))
	{
		LOGWARNING("Init");
		if (CRenderer::IsInitialised())
			g_Game->GetView()->GetMutableWaterManager().SetRenderingEnabled(true);
	}

	virtual void Deinit()
	{
	}

	virtual void Serialize(ISerializer& serialize)
	{
		serialize.NumberFixed_Unbounded("height", m_WaterHeight);
	}

	virtual void Deserialize(const CParamNode& paramNode, IDeserializer& deserialize)
	{
		Init(paramNode);

		deserialize.NumberFixed_Unbounded("height", m_WaterHeight);

		if (CRenderer::IsInitialised())
			g_Game->GetView()->GetMutableWaterManager().SetMapSize(GetSimContext().GetTerrain().GetVerticesPerSide());
	}

	virtual void HandleMessage(const CMessage& msg, bool UNUSED(global))
	{
		switch (msg.GetType())
		{
			case MT_TerrainChanged:
			{
				LOGWARNING("TerrainChange");
				// Tell the renderer to redraw part of the map.
				if (CRenderer::IsInitialised())
				{
					const CMessageTerrainChanged& msgData = static_cast<const CMessageTerrainChanged&> (msg);
					g_Game->GetView()->GetMutableWaterManager().SetMapSize(GetSimContext().GetTerrain().GetVerticesPerSide());
					g_Game->GetView()->GetMutableWaterManager().MarkDirty(msgData.i0, msgData.i1, msgData.j0, msgData.j1);
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

		LOGWARNING("SetWaterLevel");

		if (CRenderer::IsInitialised())
		{
			g_Game->GetView()->GetMutableWaterManager().SetWaterHeight(m_WaterHeight.ToFloat());
			CmpPtr<ICmpTerrain> cmpTerrain(GetSystemEntity());
			if (cmpTerrain && cmpTerrain->IsLoaded())
			{
				i32 size = cmpTerrain->GetVerticesPerSide();
				g_Game->GetView()->GetMutableWaterManager().MarkDirty(0, size, 0, size);
			}
		}

		CMessageWaterChanged msg;
		GetSimContext().GetComponentManager().BroadcastMessage(msg);
	}

	virtual entity_pos_t GetWaterLevel(entity_pos_t UNUSED(x), entity_pos_t UNUSED(z)) const
	{
		return m_WaterHeight;
	}

	virtual float GetExactWaterLevel(float UNUSED(x), float UNUSED(z)) const
	{
		return m_WaterHeight.ToFloat();
	}
};

REGISTER_COMPONENT_TYPE(WaterManager)
