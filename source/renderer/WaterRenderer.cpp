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

#include "renderer/WaterRenderer.h"

#include "graphics/ShaderManager.h"
#include "graphics/ShaderTechniquePtr.h"
#include "graphics/Water.h"
#include "lib/bits.h"
/* "lib/timer.h"*/
/* "maths/MathUtil.h"*/
#include "maths/BoundingBoxAligned.h"
#include "maths/Vector2D.h"
#include "ps/CStrInternStatic.h"
#include "ps/Game.h"
#include "ps/VideoMode.h"
#include "ps/World.h"
#include "renderer/backend/IDevice.h"
#include "renderer/Renderer.h"
#include "renderer/RenderingOptions.h"
#include "renderer/SceneRenderer.h"
#include "renderer/Renderer.h"
#include "renderer/SceneRenderer.h"
#include "renderer/TimeManager.h"
#include "renderer/WaterRData.h"

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

WaterRenderer::WaterRenderer()
{
	m_RefTextureSize = 0;


	m_RepeatPeriod = 16.0f;

	m_WaterEffects = true;
	m_WaterFancyEffects = false;
	m_WaterRealDepth = false;
	m_WaterRefraction = false;
	m_WaterReflection = false;
}

WaterRenderer::~WaterRenderer()
{
	// Cleanup if the caller messed up
	UnloadWaterTextures();


	m_FancyEffectsFramebuffer.reset();
	m_RefractionFramebuffer.reset();
	m_ReflectionFramebuffer.reset();

	m_FancyTexture.reset();
	m_FancyTextureDepth.reset();
	m_ReflFboDepthTexture.reset();
	m_RefrFboDepthTexture.reset();
}


///////////////////////////////////////////////////////////////////
// Progressive load of water textures
void WaterRenderer::LoadWaterTextures()
{
	wchar_t pathname[PATH_MAX];

	// Load diffuse grayscale images (for non-fancy water)
	for (size_t i = 0; i < ARRAY_SIZE(m_WaterTexture); ++i)
	{
		swprintf_s(pathname, ARRAY_SIZE(pathname), L"art/textures/animated/water/default/diffuse%02d.dds", (int)i+1);
		CTextureProperties textureProps(pathname);
		textureProps.SetAddressMode(
			Renderer::Backend::Sampler::AddressMode::REPEAT);

		CTexturePtr texture = g_Renderer.GetTextureManager().CreateTexture(textureProps);
		texture->Prefetch();
		m_WaterTexture[i] = texture;
	}

	// Load normalmaps (for fancy water)
	ReloadWaterNormalTexturesIfNeeded();

	// Load CoastalWaves
	{
		CTextureProperties textureProps(L"art/textures/terrain/types/water/coastalWave.png");
		textureProps.SetAddressMode(
			Renderer::Backend::Sampler::AddressMode::REPEAT);
		CTexturePtr texture = g_Renderer.GetTextureManager().CreateTexture(textureProps);
		texture->Prefetch();
		m_WaveTex = texture;
	}

	// Load Foam
	{
		CTextureProperties textureProps(L"art/textures/terrain/types/water/foam.png");
		textureProps.SetAddressMode(
			Renderer::Backend::Sampler::AddressMode::REPEAT);
		CTexturePtr texture = g_Renderer.GetTextureManager().CreateTexture(textureProps);
		texture->Prefetch();
		m_FoamTex = texture;
	}

	RecreateOrLoadTexturesIfNeeded();
}

void WaterRenderer::RecreateOrLoadTexturesIfNeeded()
{
	Renderer::Backend::IDevice* backendDevice = g_VideoMode.GetBackendDevice();

	// Use screen-sized textures for minimum artifacts.
	const size_t newRefTextureSize = round_up_to_pow2(g_Renderer.GetHeight());

	if (m_RefTextureSize != newRefTextureSize)
	{
		m_ReflectionFramebuffer.reset();
		m_ReflectionTexture.reset();
		m_ReflFboDepthTexture.reset();

		m_RefractionFramebuffer.reset();
		m_RefractionTexture.reset();
		m_RefrFboDepthTexture.reset();

		m_RefTextureSize = newRefTextureSize;
	}

	// Create reflection textures.
	const bool needsReflectionTextures =
		g_RenderingOptions.GetWaterEffects() &&
		g_RenderingOptions.GetWaterReflection();
	if (needsReflectionTextures && !m_ReflectionTexture)
	{
		m_ReflectionTexture = backendDevice->CreateTexture2D("WaterReflectionTexture",
			Renderer::Backend::ITexture::Usage::SAMPLED |
				Renderer::Backend::ITexture::Usage::COLOR_ATTACHMENT,
			Renderer::Backend::Format::R8G8B8A8_UNORM, m_RefTextureSize, m_RefTextureSize,
			Renderer::Backend::Sampler::MakeDefaultSampler(
				Renderer::Backend::Sampler::Filter::LINEAR,
				Renderer::Backend::Sampler::AddressMode::MIRRORED_REPEAT));

		m_ReflFboDepthTexture = backendDevice->CreateTexture2D("WaterReflectionDepthTexture",
			Renderer::Backend::ITexture::Usage::SAMPLED |
				Renderer::Backend::ITexture::Usage::DEPTH_STENCIL_ATTACHMENT,
			Renderer::Backend::Format::D32, m_RefTextureSize, m_RefTextureSize,
			Renderer::Backend::Sampler::MakeDefaultSampler(
				Renderer::Backend::Sampler::Filter::NEAREST,
				Renderer::Backend::Sampler::AddressMode::REPEAT));

		m_ReflectionFramebuffer = backendDevice->CreateFramebuffer("ReflectionFramebuffer",
			m_ReflectionTexture.get(), m_ReflFboDepthTexture.get(), CColor(0.5f, 0.5f, 1.0f, 0.0f));
		if (!m_ReflectionFramebuffer)
		{
			g_RenderingOptions.SetWaterReflection(false);
			UpdateQuality();
		}
	}

	// Create refraction textures.
	const bool needsRefractionTextures =
		g_RenderingOptions.GetWaterEffects() &&
		g_RenderingOptions.GetWaterRefraction();
	if (needsRefractionTextures && !m_RefractionTexture)
	{
		m_RefractionTexture = backendDevice->CreateTexture2D("WaterRefractionTexture",
			Renderer::Backend::ITexture::Usage::SAMPLED |
				Renderer::Backend::ITexture::Usage::COLOR_ATTACHMENT,
			Renderer::Backend::Format::R8G8B8A8_UNORM, m_RefTextureSize, m_RefTextureSize,
			Renderer::Backend::Sampler::MakeDefaultSampler(
				Renderer::Backend::Sampler::Filter::LINEAR,
				Renderer::Backend::Sampler::AddressMode::MIRRORED_REPEAT));

		m_RefrFboDepthTexture = backendDevice->CreateTexture2D("WaterRefractionDepthTexture",
			Renderer::Backend::ITexture::Usage::SAMPLED |
				Renderer::Backend::ITexture::Usage::DEPTH_STENCIL_ATTACHMENT,
			Renderer::Backend::Format::D32, m_RefTextureSize, m_RefTextureSize,
			Renderer::Backend::Sampler::MakeDefaultSampler(
				Renderer::Backend::Sampler::Filter::NEAREST,
				Renderer::Backend::Sampler::AddressMode::REPEAT));

		m_RefractionFramebuffer = backendDevice->CreateFramebuffer("RefractionFramebuffer",
			m_RefractionTexture.get(), m_RefrFboDepthTexture.get(), CColor(1.0f, 0.0f, 0.0f, 0.0f));
		if (!m_RefractionFramebuffer)
		{
			g_RenderingOptions.SetWaterRefraction(false);
			UpdateQuality();
		}
	}

	const uint32_t newWidth = static_cast<uint32_t>(g_Renderer.GetWidth());
	const uint32_t newHeight = static_cast<uint32_t>(g_Renderer.GetHeight());
	if (m_FancyTexture && (m_FancyTexture->GetWidth() != newWidth || m_FancyTexture->GetHeight() != newHeight))
	{
		m_FancyEffectsFramebuffer.reset();
		m_FancyTexture.reset();
		m_FancyTextureDepth.reset();
	}

	// Create the Fancy Effects textures.
	const bool needsFancyTextures =
		g_RenderingOptions.GetWaterEffects() &&
		g_RenderingOptions.GetWaterFancyEffects();
	if (needsFancyTextures && !m_FancyTexture)
	{
		m_FancyTexture = backendDevice->CreateTexture2D("WaterFancyTexture",
			Renderer::Backend::ITexture::Usage::SAMPLED |
				Renderer::Backend::ITexture::Usage::COLOR_ATTACHMENT,
			Renderer::Backend::Format::R8G8B8A8_UNORM, g_Renderer.GetWidth(), g_Renderer.GetHeight(),
			Renderer::Backend::Sampler::MakeDefaultSampler(
				Renderer::Backend::Sampler::Filter::LINEAR,
				Renderer::Backend::Sampler::AddressMode::REPEAT));

		m_FancyTextureDepth = backendDevice->CreateTexture2D("WaterFancyDepthTexture",
			Renderer::Backend::ITexture::Usage::DEPTH_STENCIL_ATTACHMENT,
			Renderer::Backend::Format::D32, g_Renderer.GetWidth(), g_Renderer.GetHeight(),
			Renderer::Backend::Sampler::MakeDefaultSampler(
				Renderer::Backend::Sampler::Filter::LINEAR,
				Renderer::Backend::Sampler::AddressMode::REPEAT));

		m_FancyEffectsFramebuffer = backendDevice->CreateFramebuffer("FancyEffectsFramebuffer",
			m_FancyTexture.get(), m_FancyTextureDepth.get());
		if (!m_FancyEffectsFramebuffer)
		{
			g_RenderingOptions.SetWaterRefraction(false);
			UpdateQuality();
		}
	}
}

void WaterRenderer::ReloadWaterNormalTexturesIfNeeded()
{
	CWater* water = g_Game->GetWorld()->GetWater();
	const CStrW& waterType = water ? water->m_WaterType : L"ocean";

	if (waterType == m_CachedWaterType && m_NormalMap[0])
		return;

	m_CachedWaterType = waterType;
	wchar_t pathname[PATH_MAX];
	for (size_t i = 0; i < ARRAY_SIZE(m_NormalMap); ++i)
	{
		swprintf_s(pathname, ARRAY_SIZE(pathname), L"art/textures/animated/water/%ls/normal00%02d.png", waterType.c_str(), static_cast<int>(i) + 1);
		CTextureProperties textureProps(pathname);
		textureProps.SetAddressMode(
			Renderer::Backend::Sampler::AddressMode::REPEAT);
		textureProps.SetAnisotropicFilter(true);

		CTexturePtr texture = g_Renderer.GetTextureManager().CreateTexture(textureProps);
		texture->Prefetch();
		m_NormalMap[i] = texture;
	}
}

///////////////////////////////////////////////////////////////////
// Unload water textures
void WaterRenderer::UnloadWaterTextures()
{
	for (size_t i = 0; i < ARRAY_SIZE(m_WaterTexture); i++)
		m_WaterTexture[i].reset();

	for (size_t i = 0; i < ARRAY_SIZE(m_NormalMap); i++)
		m_NormalMap[i].reset();

	m_RefractionFramebuffer.reset();
	m_ReflectionFramebuffer.reset();

	m_ReflectionTexture.reset();
	m_RefractionTexture.reset();
}

void WaterRenderer::RenderWaves(
	Renderer::Backend::IDeviceCommandContext* deviceCommandContext,
	WaterRData& renderData,
	const CFrustum& frustrum)
{
	if (!m_WaterFancyEffects)
		return;

	GPU_SCOPED_LABEL(deviceCommandContext, "Render Waves");

	deviceCommandContext->SetGraphicsPipelineState(
		Renderer::Backend::MakeDefaultGraphicsPipelineStateDesc());
	deviceCommandContext->BeginFramebufferPass(m_FancyEffectsFramebuffer.get());
	deviceCommandContext->ClearFramebuffer();

	CShaderTechniquePtr tech = g_Renderer.GetShaderManager().LoadEffect(str_water_waves);
	deviceCommandContext->SetGraphicsPipelineState(
		tech->GetGraphicsPipelineStateDesc());
	deviceCommandContext->BeginPass();
	Renderer::Backend::IShaderProgram* shader = tech->GetShader();

	m_WaveTex->UploadBackendTextureIfNeeded(deviceCommandContext);
	m_FoamTex->UploadBackendTextureIfNeeded(deviceCommandContext);

	deviceCommandContext->SetTexture(
		shader->GetBindingSlot(str_waveTex), m_WaveTex->GetBackendTexture());
	deviceCommandContext->SetTexture(
		shader->GetBindingSlot(str_foamTex), m_FoamTex->GetBackendTexture());

	const double time = g_Renderer.GetTimeManager().GetGlobalTime();

	deviceCommandContext->SetUniform(
		shader->GetBindingSlot(str_time), static_cast<float>(time));
	const CMatrix3D transform =
		g_Renderer.GetSceneRenderer().GetViewCamera().GetViewProjection();
	deviceCommandContext->SetUniform(
		shader->GetBindingSlot(str_transform), transform.AsFloatArray());

	const std::vector<std::unique_ptr<WaveObject>>& shoreWaves = renderData.GetShoreWavesData();
	const CVertexBufferManager::Handle& indices = renderData.GetShoreWavesIndices();

	for (size_t i = 0; i < shoreWaves.size(); ++i)
	{
		if (!frustrum.IsBoxVisible(shoreWaves[i]->m_AABB))
			continue;

		CVertexBuffer::VBChunk* VBchunk = shoreWaves[i]->m_VBVertices.Get();
		VBchunk->m_Owner->UploadIfNeeded(deviceCommandContext);
		indices->m_Owner->UploadIfNeeded(deviceCommandContext);

		const uint32_t stride = sizeof(SWavesVertex);
		const uint32_t firstVertexOffset = VBchunk->m_Index * stride;

		deviceCommandContext->SetVertexAttributeFormat(
			Renderer::Backend::VertexAttributeStream::POSITION,
			Renderer::Backend::Format::R32G32B32_SFLOAT,
			offsetof(SWavesVertex, m_BasePosition), stride,
			Renderer::Backend::VertexAttributeRate::PER_VERTEX, 0);
		deviceCommandContext->SetVertexAttributeFormat(
			Renderer::Backend::VertexAttributeStream::NORMAL,
			Renderer::Backend::Format::R32G32_SFLOAT,
			offsetof(SWavesVertex, m_PerpVect), stride,
			Renderer::Backend::VertexAttributeRate::PER_VERTEX, 0);
		deviceCommandContext->SetVertexAttributeFormat(
			Renderer::Backend::VertexAttributeStream::UV0,
			Renderer::Backend::Format::R8G8_UINT,
			offsetof(SWavesVertex, m_UV), stride,
			Renderer::Backend::VertexAttributeRate::PER_VERTEX, 0);

		deviceCommandContext->SetVertexAttributeFormat(
			Renderer::Backend::VertexAttributeStream::UV1,
			Renderer::Backend::Format::R32G32B32_SFLOAT,
			offsetof(SWavesVertex, m_ApexPosition), stride,
			Renderer::Backend::VertexAttributeRate::PER_VERTEX, 0);
		deviceCommandContext->SetVertexAttributeFormat(
			Renderer::Backend::VertexAttributeStream::UV2,
			Renderer::Backend::Format::R32G32B32_SFLOAT,
			offsetof(SWavesVertex, m_SplashPosition), stride,
			Renderer::Backend::VertexAttributeRate::PER_VERTEX, 0);
		deviceCommandContext->SetVertexAttributeFormat(
			Renderer::Backend::VertexAttributeStream::UV3,
			Renderer::Backend::Format::R32G32B32_SFLOAT,
			offsetof(SWavesVertex, m_RetreatPosition), stride,
			Renderer::Backend::VertexAttributeRate::PER_VERTEX, 0);

		deviceCommandContext->SetUniform(
			shader->GetBindingSlot(str_translation), shoreWaves[i]->m_TimeDiff);
		deviceCommandContext->SetUniform(
			shader->GetBindingSlot(str_width), static_cast<float>(shoreWaves[i]->m_Width));

		deviceCommandContext->SetVertexBuffer(
			0, VBchunk->m_Owner->GetBuffer(), firstVertexOffset);
		deviceCommandContext->SetIndexBuffer(indices->m_Owner->GetBuffer());

		const uint32_t indexCount = (shoreWaves[i]->m_Width - 1) * (7 * 6);
		deviceCommandContext->DrawIndexed(indices->m_Index, indexCount, 0);

		g_Renderer.GetStats().m_DrawCalls++;
		g_Renderer.GetStats().m_WaterTris += indexCount / 3;
	}
	deviceCommandContext->EndPass();
	deviceCommandContext->EndFramebufferPass();
}

////////////////////////////////////////////////////////////////////////
// This will set the bools properly
void WaterRenderer::UpdateQuality()
{
	if (g_RenderingOptions.GetWaterEffects() != m_WaterEffects)
	{
		m_WaterEffects = g_RenderingOptions.GetWaterEffects();
		m_NeedsReloading = true;
	}
	if (g_RenderingOptions.GetWaterFancyEffects() != m_WaterFancyEffects)
	{
		m_WaterFancyEffects = g_RenderingOptions.GetWaterFancyEffects();
		m_NeedsReloading = true;
	}
	if (g_RenderingOptions.GetWaterRealDepth() != m_WaterRealDepth)
	{
		m_WaterRealDepth = g_RenderingOptions.GetWaterRealDepth();
		m_NeedsReloading = true;
	}
	if (g_RenderingOptions.GetWaterRefraction() != m_WaterRefraction)
	{
		m_WaterRefraction = g_RenderingOptions.GetWaterRefraction();
		m_NeedsReloading = true;
	}
	if (g_RenderingOptions.GetWaterReflection() != m_WaterReflection)
	{
		m_WaterReflection = g_RenderingOptions.GetWaterReflection();
		m_NeedsReloading = true;
	}
}

bool WaterRenderer::WillRenderFancyWater() const
{
	return g_VideoMode.GetBackend() != CVideoMode::Backend::GL_ARB &&
		g_RenderingOptions.GetWaterEffects();
}

size_t WaterRenderer::GetCurrentTextureIndex(const double time, const double period) const
{
	ENSURE(period > 0.0);
	return static_cast<size_t>(time * ARRAY_SIZE(m_WaterTexture) / period) % ARRAY_SIZE(m_WaterTexture);
}

size_t WaterRenderer::GetNextTextureIndex(const double time, const double period) const
{
	ENSURE(period > 0.0);
	return (GetCurrentTextureIndex(time, period) + 1) % ARRAY_SIZE(m_WaterTexture);
}

float WaterRenderer::GetWaterPlaneHeight() const
{
	return g_Game->GetWorld()->GetWater()->GetWaterHeight();
}
