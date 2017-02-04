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

	class MeshRender : public SceneObject{
	private:
		SurfaceMesh& mesh;
		VertexArrayObject vao;
		ArrayBuffer<Vec3> vertexbuffer;
		ArrayBuffer<Vec3> normalbuffer;
		ArrayBuffer<Vec3> barycbuffer;

		std::vector<GLuint> minVerticesBuffers;
		std::vector<VertexArrayObject> maxVerticesBuffers;

		bool shouldRender;

	private:
		const GLchar* vshader = R"GLSL(
        #version 330 core
        uniform mat4 M;    
        uniform mat4 MV; 
        uniform mat4 MVP; 
        in vec3 vbaryc;      ///< per-vertex barycentric
        in vec3 vposition;   ///< per-vertex position
        in vec3 vnormal;     ///< per-vertex normal
        out vec3 fnormal;    ///< per-fragment normal
        out vec3 fbaryc;     ///< per-fragment barycentric
		in vec3 fColour;
		out vec3 colour;


        void main(){ 
            gl_Position = MVP * vec4(vposition, 1.0); 
            fnormal = normalize( inverse(transpose(mat3(MV))) * vnormal );
            fbaryc = vbaryc;
colour = fColour;
        }
    )GLSL";

		const char* fshader = R"GLSL(
    #version 330 core
    // uniform vec3 LDIR; ///< TODO: fix me
    in vec3 fnormal; ///< normal camera coords
    in vec3 colour;
    out vec4 FragColor;


    void main(){
        vec3 LDIR = vec3(0,1,1);
        vec3 ldir = normalize(LDIR);
        float albedo = max( dot( normalize(fnormal), ldir ), 0 );   
        FragColor = vec4(colour*albedo, 1);    
	//	FragColor = vec4(colour,1);    
        // FragColor = vec4(fnormal,1); ///< normal map
    }
)GLSL";

	public:
		MeshRender(SurfaceMesh& mesh) : mesh(mesh){}

		void setShouldRender()
		{
			shouldRender = !shouldRender;
		}
	
		void init(){
			shouldRender = true;
			///--- Shader
			program.add_vshader_from_source(vshader);
			program.add_fshader_from_source(fshader);
			program.link();

			///--- Data
			init_data();

			///--- Attributes
			program.bind();
			vao.bind();
			program.set_attribute("vposition", vertexbuffer);
			program.set_attribute("vnormal", normalbuffer);
			program.set_attribute("vbaryc", barycbuffer);
			vao.release();
			program.release();
		}

		void init_data(){
			CHECK(mesh.is_triangle_mesh());
			CHECK(program.is_valid());

			std::vector<Vec3> v_tri; ///< per-vertex positions
			std::vector<Vec3> n_tri; ///< per-vertex (flat) normals
			std::vector<Vec3> b_tri; ///< per-vertex barycentric [1,0,0]

			auto vpoints = mesh.get_vertex_property<Vec3>("v:point");
			auto fnormals = mesh.get_face_property<Vec3>("f:normal");
			CHECK(fnormals);
			CHECK(vpoints);

			std::vector<Vec3> baryc(3);
			baryc[0] = Vec3(1, 0, 0);
			baryc[1] = Vec3(0, 1, 0);
			baryc[2] = Vec3(0, 0, 1);

			///--- Splits mesh in independent triangles
			{
				for (auto f : mesh.faces()){
					Vec3 fnormal = fnormals[f];
					int v_idx = 0; ///< cycles over baryc
					for (auto v : mesh.vertices(f)){
						v_tri.push_back(vpoints[v]);
						/// slightly wasteful, can improve?
						n_tri.push_back(fnormal);
						b_tri.push_back(baryc[v_idx++]);
					}
				}
			}

			vertexbuffer.upload(v_tri);
			normalbuffer.upload(n_tri);
			barycbuffer.upload(b_tri);




		}

		Box3 bounding_box(){ return OpenGP::bounding_box(mesh); }

		void display(){
			if (shouldRender)
			{

				program.bind();
				vao.bind();
		
				Vec3 Colour(0, 1, 0);
				program.set_attribute("fColour", Colour);
				glDrawArrays(GL_TRIANGLES, 0, mesh.n_faces() * 3 /*#verts*/);


				//render min features
				Colour = Vec3(0, 0, 1);
				program.set_attribute("fColour", Colour);

				vao.release();
				program.release();
			}
		}
	};

	//=============================================================================
} // namespace OpenGP
//=============================================================================