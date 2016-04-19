#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include "ArcballWindow.h"
#include "FeatureExtractor.h"
#include "Curvature.h"
#include "MeshRender.h"
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderVertexNormals.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderVertexNormals.h>
#include "CurveDirRenderer.h"
#include "CurveDirRenderMin.h"

using namespace OpenGP;

bool renderSmooth = true;
struct MainWindow : public ArcballWindow{
    SurfaceMesh mesh;
	FeatureExtractor extractor = FeatureExtractor(mesh);
    MeshRender renderer = MeshRender(mesh);
	SurfaceMeshRenderFlat flatRenderer = SurfaceMeshRenderFlat(mesh);
	CurveDirRender curve1Renderer = CurveDirRender(mesh);
	CurveDirRenderMin curve2Renderer = CurveDirRenderMin(mesh);
	
	
	
    MainWindow(int argc, char** argv) : ArcballWindow(__FILE__,600,600){
        if(argc!=2) mFatal("application requires one parameter! e.g. sphere.obj");
        bool success = mesh.read(argv[1]);
        if(!success) mFatal() << "File not found: " << argv[1];
	
        mesh.update_face_normals(); ///< shading
       
        extractor.init();
		std::cout << "press '1' to toggle rendering curvature directions" << std::endl;
		//render curve directions 
		this->scene.add(curve1Renderer); //render max curvatures
		this->scene.add(curve2Renderer); //render min curvatures
		this->scene.add(renderer);
//		this->scene.add(flatRenderer);

	
    }
    
    void key_callback(int key, int scancode, int action, int mods) override{
        ArcballWindow::key_callback(key, scancode, action, mods);
        if(key==GLFW_KEY_SPACE && action==GLFW_RELEASE){
    //        extractor.exec();
            mesh.update_face_normals();
			mesh.update_vertex_normals();
       //     renderer.init_data();
        }
		if (key == GLFW_KEY_S && action == GLFW_RELEASE)
		{
			//	renderer.setSmooth();
			this->scene.objects.clear();


		}
		if (key == GLFW_KEY_0)
		{//SHOW no CURVATURES
		//	renderer.renderCurvature(0);
		}
		if (key == GLFW_KEY_1 && action == GLFW_RELEASE)
		{//SHOW CURVATURES
			curve1Renderer.shouldRender();
			curve2Renderer.shouldRender();
		
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
