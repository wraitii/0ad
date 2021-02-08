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

/*
 * Water settings (speed, height) and texture management
 */

#ifndef INCLUDED_WATERRENDERER
#define INCLUDED_WATERRENDERER

#include "graphics/ShaderProgramPtr.h"
#include "graphics/Texture.h"
#include "lib/ogl.h"
#include "maths/Matrix3D.h"
#include "renderer/VertexBufferManager.h"

class CSimulation2;
class CFrustum;
class TerrainRenderer;
class WaterManager;

struct CoastalPoint;
struct WaveObject;

/**
 * This maintains a set of graphics-related things for the TerrainRenderer.
 * TODO: there is probably more that could be migrated here.
 */
class WaterRendering
{
	friend class TerrainRenderer;
	friend class WaterManager;
public:
	WaterRendering();
	~WaterRendering();

	/**
	 * Get a texturePtr to the water texture at the corresponding time point.
	 * @param advance - how many frames ahead to return.
	 */
	CTexturePtr GetTexture(float time, int advance = 0) const;

	/**
	 * The normals used for fancy water rendering depend on the graphics.
	 */
	void LoadNormalTextures(const std::wstring& type);

	/**
	 * Do the actual wave rendering.
	 * (the water plane rendering is currently done in TerrainRenderer)
	 */
	void RenderWaves(const CFrustum& frustrum);

	/**
	 * Reconstruct the screen-size dependent textures.
	 */
	void OnRendererResize();

	/**
	 * Size of the texture to render refractions/reflections in.
	 */
	size_t GetTextureSize() const { return m_RefTextureSize; }

	// Data accessed by both TerrainRenderer and Renderer (public for convenience).
	CShaderProgramPtr fancyWaterShader;

	// Model-view-projection matrices for reflected & refracted cameras
	// (used to let the vertex shader do projective texturing)
	CMatrix3D m_ReflectionMatrix;
	CMatrix3D m_RefractionMatrix;
	CMatrix3D m_RefractionProjInvMatrix;
	CMatrix3D m_RefractionViewInvMatrix;

	// Framebuffers
	GLuint m_RefractionFbo;
	GLuint m_ReflectionFbo;
private:
	bool WillRenderFancyWater();

	/**
	 * Create rendering data for wave meshes.
	 */
	void CreateWaveMeshes();

	void PrepareTextures();
	void UnprepareTextures();

	// Shouldn't be null when anything is actually used here.
	WaterManager* m_WaterManager = nullptr;

	// Waves vertex buffers
	std::vector<WaveObject* > m_ShoreWaves;	// TODO: once we get C++11, remove pointer
	// Waves indices buffer. Only one since All Wave Objects have the same.
	CVertexBuffer::VBChunk* m_ShoreWaves_VBIndices = nullptr;

	ssize_t m_TexSize;

	CTexturePtr m_WaterTexture[60];
	CTexturePtr m_NormalMap[60];

	CTexturePtr m_WaveTex;
	CTexturePtr m_FoamTex;

	GLuint m_FancyTexture;
	GLuint m_FancyTextureDepth;
	GLuint m_ReflFboDepthTexture;
	GLuint m_RefrFboDepthTexture;

	// Reflection and refraction textures for fancy water
	GLuint m_ReflectionTexture;
	GLuint m_RefractionTexture;
	size_t m_RefTextureSize;

	// Internal framebuffer for waves and such.
	GLuint m_FancyEffectsFBO;
};


#endif // INCLUDED_WATERRENDERER
