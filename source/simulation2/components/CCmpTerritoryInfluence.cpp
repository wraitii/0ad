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
#include "ICmpTerritoryInfluence.h"

#include "simulation2/MessageTypes.h"
#include "simulation2/components/ICmpOwnership.h"
#include "simulation2/components/ICmpPlayerManager.h"
#include "simulation2/components/ICmpValueModificationManager.h"

class CCmpTerritoryInfluence final : public ICmpTerritoryInfluence
{
public:
	static void ClassInit(CComponentManager& componentManager)
	{
		componentManager.SubscribeToMessageType(MT_OwnershipChanged);
		componentManager.SubscribeToMessageType(MT_ValueModification);
	}

	DEFAULT_COMPONENT_ALLOCATOR(TerritoryInfluence)

	bool m_Root;
	u16 m_Weight;
	u32 m_Radius;

	static std::string GetSchema()
	{
		return
			"<element name='Root'>"
				"<data type='boolean'/>"
			"</element>"
			"<element name='Weight'>"
				"<data type='nonNegativeInteger'>"
					"<param name='maxInclusive'>65535</param>" // Max u16 value
				"</data>"
			"</element>"
			"<element name='Radius'>"
				"<data type='nonNegativeInteger'/>"
			"</element>";
	}

	void Init(const CParamNode& paramNode) override
	{
		m_Root = paramNode.GetChild("Root").ToBool();
		m_Weight = (u16)paramNode.GetChild("Weight").ToInt();
		m_Radius = paramNode.GetChild("Radius").ToInt();
	}

	void Deinit() override
	{
		OnChanged();
	}

	void Update(const CParamNode& paramNode) override
	{
		// TODO: only update on change
		Init(paramNode);
		OnChanged();
	}

	void Serialize(ISerializer& UNUSED(serialize)) override
	{
	}

	void Deserialize(const CParamNode& paramNode, IDeserializer& UNUSED(deserialize)) override
	{
		Init(paramNode);
	}

	void HandleMessage(const CMessage& msg, bool UNUSED(global)) override
	{
		switch (msg.GetType())
		{
		case MT_OwnershipChanged:
			{
				const CMessageOwnershipChanged& msgData = static_cast<const CMessageOwnershipChanged&> (msg);
				// handled in DeInit();
				if (msgData.to != INVALID_PLAYER)
					OnChanged();
				break;
			}
		case MT_ValueModification:
			{
				const CMessageValueModification& msgData = static_cast<const CMessageValueModification&> (msg);
				if (msgData.component == L"TerritoryInfluence")
					OnChanged();
				break;
			}
		}
	}

	void OnChanged() const
	{
		CMessageTerritoryInfluenceChanged msg(GetEntityId());
		GetSimContext().GetComponentManager().PostMessage(GetEntityId(), msg);
	}

	bool IsRoot() const override
	{
		CmpPtr<ICmpValueModificationManager> cmpValueModificationManager(GetSystemEntity());
		if (!cmpValueModificationManager)
			return m_Root;

		return cmpValueModificationManager->ApplyModifications(L"TerritoryInfluence/Root", m_Root, GetEntityId());
	}

	u16 GetWeight() const override
	{
		CmpPtr<ICmpValueModificationManager> cmpValueModificationManager(GetSystemEntity());
		if (!cmpValueModificationManager)
			return m_Weight;

		return cmpValueModificationManager->ApplyModifications(L"TerritoryInfluence/Weight", m_Weight, GetEntityId());
	}

	u32 GetRadius() const override
	{
		CmpPtr<ICmpValueModificationManager> cmpValueModificationManager(GetSystemEntity());
		if (!cmpValueModificationManager)
			return m_Radius;

		u32 newRadius = cmpValueModificationManager->ApplyModifications(L"TerritoryInfluence/Radius", m_Radius, GetEntityId());
		return newRadius;
	}
};

REGISTER_COMPONENT_TYPE(TerritoryInfluence)
