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

/*
 * Water settings (speed, height) and texture management
 */

#ifndef INCLUDED_WATERMANAGER
#define INCLUDED_WATERMANAGER

#include "graphics/Color.h"
#include "graphics/Texture.h"
#include "maths/Matrix3D.h"
#include "maths/Vector2D.h"
#include "renderer/backend/IDeviceCommandContext.h"
#include "renderer/backend/IFramebuffer.h"
#include "renderer/backend/ITexture.h"
#include "renderer/VertexBufferManager.h"

#include <memory>
#include <vector>

class CFrustum;
class CSceneRenderer;
class ShaderModelRenderer;
class TerrainRenderer;
class WaterRData;

/**
 * Class WaterManager: Maintain rendering-related water settings and textures
 * Anything that affects gameplay should go in CcmpWaterManager.cpp and passed to this (possibly as copy).
 */

class WaterManager
{
	friend class ShaderModelRenderer;
	friend class CSceneRenderer;
	friend class TerrainRenderer;
public:
	WaterManager();
	~WaterManager();

	void LoadWaterTextures();

	/**
	 * Recreates/loads needed water textures.
	 */
	void RecreateOrLoadTexturesIfNeeded();

	/**
	 * ReloadWaterNormalTextures: Reload the normal textures so that changing
	 * water type in Atlas will actually do the right thing.
	 */
	void ReloadWaterNormalTextures(const CStrW& waterType);

	/**
	 * UnloadWaterTextures: Free any loaded water textures and reset the internal state
	 * so that another call to LoadWaterTextures will begin progressive loading.
	 */
	void UnloadWaterTextures();

	/**
	 * Updates the settings to the one from the renderer, and sets m_NeedsReloading.
	 */
	void UpdateQuality();

	/**
	 * @return the height of the rendered water plane. Used for placing refraction/reflection cameras.
	 */
	float GetWaterPlaneHeight() const;

	/**
	 * Returns true if fancy water shaders will be used (i.e. the hardware is capable
	 * and it hasn't been configured off)
	 */
	bool WillRenderFancyWater() const;

	void RenderWaves(
		Renderer::Backend::IDeviceCommandContext* deviceCommandContext,
		WaterRData& renderData,
		const CFrustum& frustrum);

	/**
	 * Returns an index of the current texture that should be used for rendering
	 * water.
	 */
	size_t GetCurrentTextureIndex(const double time, const double period) const;
	size_t GetNextTextureIndex(const double time, const double period) const;

protected:
	CStrW m_CachedWaterType;


	std::unique_ptr<Renderer::Backend::ITexture> m_FancyTexture;
	std::unique_ptr<Renderer::Backend::ITexture> m_FancyTextureDepth;
	std::unique_ptr<Renderer::Backend::ITexture> m_ReflFboDepthTexture;
	std::unique_ptr<Renderer::Backend::ITexture> m_RefrFboDepthTexture;

	// True if the shaders are dirty.
	bool m_NeedsReloading;

	// If disabled, force the use of the simple water shader for rendering.
	bool m_WaterEffects;
	// Those variables register the current quality level. If there is a change, I have to recompile the shader.
	// Use real depth or use the fake precomputed one.
	bool m_WaterRealDepth;
	// Use fancy shore effects and show trails behind ships
	bool m_WaterFancyEffects;
	// Use refractions instead of simply making the water more or less transparent.
	bool m_WaterRefraction;
	// Use complete reflections instead of showing merely the sky.
	bool m_WaterReflection;

	CTexturePtr m_WaterTexture[60];
	CTexturePtr m_NormalMap[60];

	CTexturePtr m_WaveTex;
	CTexturePtr m_FoamTex;

	float m_RepeatPeriod;

	// Reflection and refraction textures for fancy water
	std::unique_ptr<Renderer::Backend::ITexture> m_ReflectionTexture;
	std::unique_ptr<Renderer::Backend::ITexture> m_RefractionTexture;
	size_t m_RefTextureSize;

	// framebuffer objects
	std::unique_ptr<Renderer::Backend::IFramebuffer> m_RefractionFramebuffer;
	std::unique_ptr<Renderer::Backend::IFramebuffer> m_ReflectionFramebuffer;
	std::unique_ptr<Renderer::Backend::IFramebuffer> m_FancyEffectsFramebuffer;

	// Model-view-projection matrices for reflected & refracted cameras
	// (used to let the vertex shader do projective texturing)
	CMatrix3D m_ReflectionMatrix;
	CMatrix3D m_RefractionMatrix;
	CMatrix3D m_RefractionProjInvMatrix;
	CMatrix3D m_RefractionViewInvMatrix;
};


#endif // INCLUDED_WATERMANAGER
