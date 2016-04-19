#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include <OpenGP/GL/SegmentsRenderer.h>
#include "ArcballWindow.h"
#include "FeatureExtractor.h"
#include "Curvature.h"
#include "MeshRender.h"
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderVertexNormals.h>
using namespace OpenGP;

bool renderSmooth = true;
struct MainWindow : public ArcballWindow{
    SurfaceMesh mesh;
	FeatureExtractor extractor = FeatureExtractor(mesh);
    MeshRender renderer = MeshRender(mesh);
	SurfaceMeshRenderFlat flatRender = SurfaceMeshRenderFlat(mesh);

	
	
    MainWindow(int argc, char** argv) : ArcballWindow(__FILE__,600,600){
        if(argc!=2) mFatal("application requires one parameter! e.g. sphere.obj");
        bool success = mesh.read(argv[1]);
        if(!success) mFatal() << "File not found: " << argv[1];
	
        mesh.update_face_normals(); ///< shading
        this->scene.add(renderer);
        extractor.init();
    }
    
    void key_callback(int key, int scancode, int action, int mods) override{
        ArcballWindow::key_callback(key, scancode, action, mods);
        if(key==GLFW_KEY_SPACE && action==GLFW_RELEASE){
            extractor.exec();
            mesh.update_face_normals();
            renderer.init_data();
			flatRender.init_data();
		//	segRender.init();
        }
		if (key == GLFW_KEY_S && action == GLFW_RELEASE)
		{
			//	renderer.setSmooth();
			this->scene.objects.clear();
			renderSmooth = !renderSmooth;
			if (renderSmooth)
			{
				this->scene.add(renderer);
			}
			else
			{
				this->scene.add(flatRender);
			}
		}
		if (key == GLFW_KEY_0)
		{//SHOW no CURVATURES
		//	renderer.renderCurvature(0);
		}
		if (key == GLFW_KEY_1 && action == GLFW_RELEASE)
		{//SHOW K1 CURVATURES
		/*	renderer.renderCurvature(1);
			SurfaceMesh::Vertex_property<Vec3> points = mesh.get_vertex_property<Vec3>("v:point");
			OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> scalar = mesh.get_vertex_property<Scalar>("v:curvature_k1");
			SurfaceMesh::Vertex_iterator vit, vend = mesh.vertices_end();
				
			for (vit = mesh.vertices_begin(); vit != vend; ++vit)
			{
				Vec3 P1 = points[*vit];
				Vec3 P2 = P1 * scalar[*vit];
				SegmentsRenderer segRender = SegmentsRenderer(P1, P2);
				this->scene.add(segRender);
			}*/
			
		}
		if (key == GLFW_KEY_2 && action == GLFW_RELEASE)
		{//SHOW K2 CURVATURES
		//	renderer.renderCurvature(2);
		}
		if (key == GLFW_KEY_3 && action == GLFW_RELEASE)
		{//SHOW PRINCIPAL CURVATURES
		//	renderer.renderCurvature(3);
		}
		
    }
};

int main(int argc, char** argv){
    MainWindow window(argc, argv);


    return window.run();
}
