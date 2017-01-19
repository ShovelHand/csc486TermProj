#pragma once
#include <vector>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>
#include <OpenGP/GL/glfw.h>
#include <OpenGP/GL/check_error_gl.h>
#include <OpenGP/MLogger.h>

#include <OpenGP/GL/Buffer.h>
#include <OpenGP/GL/SceneGraph.h>
#include <OpenGP/SurfaceMesh/bounding_box.h>

//=============================================================================
namespace OpenGP {
	//=============================================================================

	class FeatureRenderer : public SceneObject{
	private:
		SurfaceMesh& mesh;

		VertexArrayObject _vao;           ///< OpenGL object storing data/attributes
		MatMxN _data;                     ///< reference to data to be rendered  
		ArrayBuffer<Vec3> _buffer_vpos;   ///< per-vertex position
		ArrayBuffer<Vec3> _buffer_vcolor; ///< per-vertex color (optional)

		std::vector<Vec3> segmentPoints;

		std::vector<GLuint> minVerticesBuffers;
		std::vector<VertexArrayObject> maxVerticesBuffers;
		OpenGP::SurfaceMesh::Vertex_property<OpenGP::Vec3> k_dir;
		bool bRenderLines;
		bool bRenderPoints;
	private:

		const GLchar* vshader = R"GLSL( 
        #version 330
        uniform mat4 M;    
        uniform mat4 MV; 
        uniform mat4 MVP;
        in vec3 vpoint;
        in vec3 vcolor;
        out vec3 fcolor;
        void main() {  
            gl_Position = MVP * vec4(vpoint, 1.0);
            fcolor = vcolor;
        }
    )GLSL";

		const char* fshader = R"GLSL(
        #version 330
        in vec3 fcolor;
        out vec4 color;
        void main(){ color = vec4(fcolor,1); }        
    )GLSL";


	public:
		FeatureRenderer(SurfaceMesh& mesh) : mesh(mesh){ }

		void setSegmentPoints(std::vector<Vec3> vec) { segmentPoints = vec; }
		void shouldRenderLines(){ bRenderLines = !bRenderLines; }
		void shouldRenderPoints(){ bRenderPoints = !bRenderPoints; }

		void init(){
			bRenderLines = true;
			bRenderPoints = true;
			if (!segmentPoints.size() > 0)
			{
				std::cout << "no ridgeline points given" << std::endl;
				return;
			}
				

			program.add_vshader_from_source(vshader);
			program.add_fshader_from_source(fshader);
			program.link();
			
	
			///--- Data
			_buffer_vpos.upload(segmentPoints);

			///--- Data
			init_data();

			///--- Attributes
			program.bind();
			_vao.bind();
			program.set_attribute("vpoint", _buffer_vpos);

			_vao.release();
			program.release();
		}

		void init_data()
		{


		}

		Box3 bounding_box(){ return OpenGP::bounding_box(mesh); }

		void display()
		{
			
				//if (_data.cols() == 0) return;
				_vao.bind();
				program.bind();
				glLineWidth(2.0f);

				Vec3 color(1, 0, 1);
				program.set_attribute("vcolor", color); ///< default use object color
				if (bRenderLines)
				{	
					glDrawArrays(GL_LINES, 0, segmentPoints.size());
				}
				if (bRenderPoints)
				{
					glPointSize(5.0);
					glDrawArrays(GL_POINTS, 0, segmentPoints.size());

				}
				_vao.release();
				program.release();
			

		}
	};

	//=============================================================================
} // namespace OpenGP
//=============================================================================
