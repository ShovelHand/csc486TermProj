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
	//getters
	std::vector<Vec3> GetSingleTriangles(){ return singleTriangles; }
	std::vector<Vec3> GetRidgeLinePoints(){ return ridgeLinePoints; }

private:
	//these will probably get removed from the final project, but for now are great for checking if curvature is computed well
	//from lab3 curvature
	void visualize_k1_curvature();
	void visualize_k2_curvature();

//	void ComputeCurvatures();
	void ComputeVertexShapeOperators();
	OpenGP::Mat3x3 ComputeEdgeShapeOperators(SurfaceMesh::Vertex_iterator vertex);
	void ComputeMaxMinCurvatures();
	void BuildLinearFunctions();
	void CorrectCurvatureSigns();
	void ComputeFeatureLines();
	void ProcessSingularTriangles();
	Vec3 ComputeRidgePoint(SurfaceMesh::Vertex_iterator v1, SurfaceMesh::Vertex_iterator v2);
	
	SurfaceMesh& mesh;
	SurfaceMesh::Vertex_property<Vec3> vpoints;
	SurfaceMesh::Face_property<Vec3> fnormals;
	const float min_cos = std::cos(0.25*M_PI); ///< min angle (avoid face foldover)

	//OpenGP::SurfaceMesh::Vertex_property<OpenGP::Point> vpoint;
	OpenGP::SurfaceMesh::Vertex_property<float> vquality;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> varea;
	
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Vec3> vdirection_kmax;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Vec3> vdirection_kmin;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> vcurvature_kmax;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> vcurvature_kmin;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Mat3x3> vShapeOperator;
	OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> vextremality;

	OpenGP::SurfaceMesh::Halfedge_property<OpenGP::Vec3> hface_norm;
	OpenGP::SurfaceMesh::Halfedge_property<bool> hTraversed_this_time;
	OpenGP::SurfaceMesh::Edge_property<OpenGP::Mat3x3> eShapeOperator;

	OpenGP::SurfaceMesh::Face_property<bool> fis_regular;

	//stuff we will want to render
	std::vector<Vec3> singleTriangles; //we might want to render these for fun.
	std::vector<Vec3> ridgeLinePoints;

};
