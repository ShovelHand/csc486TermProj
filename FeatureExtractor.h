#pragma once 
#include <OpenGP/SurfaceMesh/Surfacemesh.h>
#include "Quadric.h"
#include "PriorityQueue.h"

/*this is the class to extract features for my csc486 term project on highlighting a mesh's extreme and minimal surface features*/
class FeatureExtractor{
    typedef SurfaceMesh::Halfedge Halfedge;
    typedef SurfaceMesh::Vertex   Vertex;
    typedef SurfaceMesh::Face     Face;
public:
	FeatureExtractor(SurfaceMesh& mesh); 
	~FeatureExtractor();

	void init();
	void exec();

private:
	//these will probably get removed from the final project, but for now are great for checking if curvature is computed well
	//from lab3 curvature
	void create_colours(OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> prop);
	void visualize_k1_curvature();
	void visualize_k2_curvature();

	void ComputeCurvatures();
	void BuildExtremeCoeffs();
	
	SurfaceMesh& mesh;
	SurfaceMesh::Vertex_property<Vec3> vpoints;
	SurfaceMesh::Face_property<Vec3> fnormals;
	const float min_cos = std::cos(0.25*M_PI); ///< min angle (avoid face foldover)

	//OpenGP::SurfaceMesh::Vertex_property<OpenGP::Point> vpoint;
	//OpenGP::SurfaceMesh::Vertex_property<float> vquality;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> varea;
	OpenGP::SurfaceMesh::Edge_property<OpenGP::Scalar> ecotan;
	
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> vcurvature_K;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> vcurvature_H;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> vcurvature_k1;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> vcurvature_k2;

};
