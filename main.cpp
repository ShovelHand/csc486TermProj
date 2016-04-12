#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include "ArcballWindow.h"
#include "FeatureExtractor.h"
#include "Curvature.h"
#include "MeshRender.h"

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
        }
		if (key == GLFW_KEY_S && action == GLFW_RELEASE){
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
    }
};

int main(int argc, char** argv){
    MainWindow window(argc, argv);

	

    return window.run();
}
