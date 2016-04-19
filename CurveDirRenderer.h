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

	class CurveDirRender : public SceneObject{
	private:
		SurfaceMesh& mesh;

		VertexArrayObject _vao;           ///< OpenGL object storing data/attributes
		MatMxN _data;                     ///< reference to data to be rendered  
		ArrayBuffer<Vec3> _buffer_vpos;   ///< per-vertex position
		ArrayBuffer<Vec3> _buffer_vcolor; ///< per-vertex color (optional)

		std::vector<GLuint> minVerticesBuffers;
		std::vector<VertexArrayObject> maxVerticesBuffers;
		OpenGP::SurfaceMesh::Vertex_property<OpenGP::Vec3> k_dir;
		bool bRender;

	private:

		void load(const MatMxN &P1, const MatMxN &P2){
			CHECK(P1.cols() == P2.cols());
			CHECK((P1.rows() == 3) && (P2.rows() == 3));
			_data.resize(3, 2 * P1.cols());
			for (int i = 0; i<P1.cols(); i++){
				_data.col(i * 2 + 0) = P1.col(i);
				_data.col(i * 2 + 1) = P2.col(i);
			}
		}
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
		CurveDirRender(SurfaceMesh& mesh) : mesh(mesh){ }

		void shouldRender(){ bRender = !bRender; }

		void init(){
			bRender = false;
			program.add_vshader_from_source(vshader);
			program.add_fshader_from_source(fshader);
			program.link();
			//load point matrices
			Mat3xN P1 = OpenGP::vertices_matrix(mesh);
			Mat3xN N = OpenGP::normals_matrix(mesh); //'cause I can't figure out how to set number of rows
			
		
			k_dir = mesh.get_vertex_property<Vec3>("v:direction_kmax");

			int i = 0;
			for (const auto& vertex : mesh.vertices())
			{
				N.row(0).col(i) << k_dir[vertex].x();
				N.row(1).col(i) << k_dir[vertex].y();
				N.row(2).col(i) << k_dir[vertex].z();
				++i;
			}
			Mat3xN P2;
			
			P2 = P1 + 0.2*N;
			
			load(P1, P2);
			///--- Data
			_buffer_vpos.upload(_data.data(), _data.cols(), sizeof(Vec3));

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
			if (bRender)
			{
				if (_data.cols() == 0) return;
				_vao.bind();
				program.bind();
				glLineWidth(1.0f);
			
				Vec3 color(1, 0, 0);
		
				program.set_attribute("vcolor", color); ///< default use object color
				glDrawArrays(GL_LINES, 0, _data.cols());
				_vao.release();
				program.release();
			}		
		}
	};

	//=============================================================================
} // namespace OpenGP
//=============================================================================
