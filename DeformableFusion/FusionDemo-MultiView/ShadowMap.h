#pragma once

#define OGLPLUS_NO_DEFAULTED_FUNCTIONS 1
#define OGLPLUS_NO_VARIADIC_TEMPLATES 1
#define OGLPLUS_NO_INHERITED_CONSTRUCTORS 1
#define OGLPLUS_NO_NOEXCEPT 1

#define OGLPLUS_USE_GLEW 1
#define OGLPLUS_USE_BOOST_CONFIG 0
#define OGLPLUS_NO_SITE_CONFIG 1
//#include <oglplus/gl.hpp>
#include <oglplus/all.hpp>
#include <oglplus/opt/list_init.hpp>
#include <oglplus/shapes/cube.hpp>
#include <oglplus/bound/texture.hpp>
#include <oglplus/bound/framebuffer.hpp>
#include <oglplus/bound/renderbuffer.hpp>
#include <oglplus/math/matrix.hpp>

#define MULTISAMPLING 1

#if MULTISAMPLING == 1
#define SCREEN_DEPTH_TARGET Texture::Target::_2DMultisample
#else
#define SCREEN_DEPTH_TARGET Texture::Target::_2D
#endif

struct ShadowMap
{
	static bool const disabled = false;
	static bool const debug_draw = false;

	static std::string readFile( std::string const & filename )
	{
		std::ifstream t(filename.c_str());
		return std::string((std::istreambuf_iterator<char>(t)),
		    				std::istreambuf_iterator<char>());
	}

	ShadowMap( char const * path = "" )
	{
		if (disabled)
			return;

		using namespace oglplus;
		std::string vsShaderPath = std::string(path) + "deferred_shadows_vs.txt";
		std::string fsShaderPath = std::string(path) + "deferred_shadows_fs.txt";
		if (!std::filesystem::exists(vsShaderPath) || !std::filesystem::exists(fsShaderPath))
		{
			std::cerr << "Error: missing Shader files: " << vsShaderPath << " OR " << fsShaderPath << std::endl;
			return;
		}

		deferred_shadows_vs.Source(readFile(vsShaderPath));
		deferred_shadows_vs.Compile();

		deferred_shadows_fs.Source(readFile(fsShaderPath));
		deferred_shadows_fs.Compile();

		deferred_shadows_prog.AttachShader(deferred_shadows_vs);
		deferred_shadows_prog.AttachShader(deferred_shadows_fs);
		deferred_shadows_prog.Link();
		deferred_shadows_prog.Use();

        Texture::Active(0);
        gl.Bound(Texture::Target::_2D, tex_color)
		.MinFilter(TextureMinFilter::Nearest)
		.MagFilter(TextureMagFilter::Nearest)
		.WrapS(TextureWrap::ClampToEdge)
		.WrapT(TextureWrap::ClampToEdge)
		.Image2D(
			0,
			PixelDataInternalFormat::RGBA,
			shadow_map_size, shadow_map_size,
			0,
			PixelDataFormat::RGBA,
			PixelDataType::UnsignedByte,
			nullptr
			);

		gl.Bound(Texture::Target::_2D, tex_depth)
			.MinFilter(TextureMinFilter::Linear)
			.MagFilter(TextureMagFilter::Linear)
			.WrapS(TextureWrap::ClampToEdge)
			.WrapT(TextureWrap::ClampToEdge)
			//.CompareMode(TextureCompareMode::CompareRefToTexture)
			.Image2D(
				0,
				PixelDataInternalFormat::DepthComponent,
				shadow_map_size, shadow_map_size,
				0,
				PixelDataFormat::DepthComponent,
				PixelDataType::Float,
				nullptr
			);

		gl.Bound(Framebuffer::Target::Draw, fbo)
		.AttachTexture(
			FramebufferAttachment::Color,
			tex_color,
			0
			)
		.AttachTexture(
			FramebufferAttachment::Depth,
			tex_depth,
			0
			);

		gl.Bind(Framebuffer::Target::Draw, dfb);

		glUseProgram(0);
	}

	void shadow_pass_begin()
	{
		if (disabled)
			return;

		using namespace oglplus;

		fbo.Bind(Framebuffer::Target::Draw);
		gl.Viewport(shadow_map_size, shadow_map_size);
		gl.ClearDepth(1.0f);
		gl.ClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		gl.Clear().ColorBuffer().DepthBuffer();

		float data[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, data);
		Mat4f modelview(data);
		modelview = Transposed(modelview);

		glGetFloatv(GL_PROJECTION_MATRIX, data);
		Mat4f projection(data);
		projection = Transposed(projection);

		shadow_map_mvp = projection * modelview;

		//glPolygonOffset(1.1, 4);
	}

	void shadow_pass_end()
	{
		if (disabled)
			return;

		using namespace oglplus;

		gl.Bind(Framebuffer::Target::Draw, dfb);

		//glPolygonOffset(0, 0);
	}

	void snap_screen_depth( int x, int y, int width, int height )
	{
		if (disabled)
			return;

		using namespace oglplus;

		if (!tex_screen_depth || tex_screen_depth->Width(SCREEN_DEPTH_TARGET) != width || tex_screen_depth->Height(SCREEN_DEPTH_TARGET) != height)
		{
			fbo_screen_depth.reset(new Framebuffer);
			tex_screen_depth.reset(new Texture);

			if (MULTISAMPLING == 1)
			{
				gl.Bound( SCREEN_DEPTH_TARGET, *tex_screen_depth)
				.Image2DMultisample(
					4,
					PixelDataInternalFormat::DepthComponent,
					width, height,
					true
				);
			}
			else
			{
				gl.Bound( SCREEN_DEPTH_TARGET, *tex_screen_depth)
					.MinFilter(TextureMinFilter::Nearest)
					.MagFilter(TextureMagFilter::Nearest)
					.WrapS(TextureWrap::ClampToEdge)
					.WrapT(TextureWrap::ClampToEdge)
					.Image2D(
						0,
						PixelDataInternalFormat::DepthComponent,
						width, height,
						0,
						PixelDataFormat::DepthComponent,
						PixelDataType::Float,
						nullptr
					);
			}



			gl.Bound(Framebuffer::Target::Draw, *fbo_screen_depth)
				.AttachTexture(
					FramebufferAttachment::Depth,
					*tex_screen_depth,
					0
					);
			gl.Bind(Framebuffer::Target::Draw, dfb);
		}

		//gl.Bound(Texture::Target::_2D, *tex_screen_depth).CopySubImage2D(0, 0, 0, x, y, width, height);
		//gl.Bind(Framebuffer::Target::Read, dfb);
		gl.Bind(Framebuffer::Target::Draw, *fbo_screen_depth);
		glBlitFramebuffer(x, y, x + width, y + height, 0, 0, width, height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
		gl.Bind(Framebuffer::Target::Draw, dfb);
	}

	void apply_deferred_shadows()
	{
		if (disabled)
			return;

		using namespace oglplus;

		gl.Disable(Capability::DepthTest);
		gl.Disable(Capability::CullFace);
		glEnable(GL_DEPTH_CLAMP);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

					

		Texture::Active(1);
		gl.Bound(Texture::Target::_2D, tex_depth).CompareMode(TextureCompareMode::CompareRefToTexture);
		tex_depth.Bind(TextureTarget::_2D);

		Texture::Active(0);
		tex_screen_depth->Bind(SCREEN_DEPTH_TARGET);

		deferred_shadows_prog.Use();
		Uniform<Mat4f>(deferred_shadows_prog, "shadow_map_mvp").Set(shadow_map_mvp);

		glDrawArrays(GL_QUADS, 0, 4);

		glDisable(GL_DEPTH_CLAMP);
		gl.Enable(Capability::DepthTest);
		gl.Enable(Capability::CullFace);
		glDisable(GL_BLEND);

		gl.Bound(Texture::Target::_2D, tex_depth).CompareMode(TextureCompareMode::None);

		glUseProgram(0);
	}


	void draw_texture( oglplus::Texture & texture, float x, float y, float size_x, float size_y )
	{
		glDrawTextureNV(oglplus::GetGLName(texture), 0, x, y, x + size_x, y + size_y, 0.f, 0, 0, 1, 1);
	}

	void draw_textures()
	{
		if (disabled)
			return;

		if (!debug_draw)
			return;

		draw_texture( tex_color, 100, 0, 100, 100 );
		draw_texture( tex_depth, 200, 0, 100, 100 );
		if (tex_screen_depth)
			draw_texture( *tex_screen_depth, 00, 0, 100, 100 );
	}

	// wrapper around the current OpenGL context
	oglplus::Context gl;

	oglplus::Framebuffer  fbo;
	oglplus::DefaultFramebuffer dfb;
	oglplus::Texture tex_color;
	oglplus::Texture tex_depth;

	std::unique_ptr<oglplus::Framebuffer> fbo_screen_depth;
	std::unique_ptr<oglplus::Texture>	  tex_screen_depth;

	static const int shadow_map_size = 1024;

	oglplus::VertexShader   deferred_shadows_vs;
    oglplus::FragmentShader deferred_shadows_fs;
	oglplus::Program		deferred_shadows_prog;

	oglplus::Mat4f          shadow_map_mvp;
};
