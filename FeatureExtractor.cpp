#include "FeatureExtractor.h"
#include <OpenGP/MLogger.h>
#include <math.h>

FeatureExtractor::FeatureExtractor(SurfaceMesh& mesh) : mesh(mesh)
{
	vpoints = mesh.vertex_property<Point>("v:point");
	fnormals = mesh.face_property<Normal>("f:normal");
	varea = mesh.add_vertex_property<OpenGP::Scalar>("v:area");
	ecotan = mesh.add_edge_property<OpenGP::Scalar>("e:cotan");
	vcurvature_K = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_K");
	vcurvature_H = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_H");
	vcurvature_k1 = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_k1");
	vcurvature_k2 = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_k2");
}
FeatureExtractor::~FeatureExtractor()
{
	mesh.remove_vertex_property(varea);
	//mesh.remove_vertex_property(ecotan);
	mesh.remove_vertex_property(vcurvature_K);
	mesh.remove_vertex_property(vcurvature_H);
	mesh.remove_vertex_property(vcurvature_k1);
	mesh.remove_vertex_property(vcurvature_k2);
	mesh.garbage_collection();
}

void FeatureExtractor::ComputeCurvatures()
{
	//compute mean curvatures
	for (auto const& vertex : mesh.vertices()) {
		varea[vertex] = 0.0;
	}
	SurfaceMesh::Vertex_around_face_circulator vFit;
	SurfaceMesh::Vertex v0, v1, v2;
	Scalar a;

	//compute gaussian curvature for each vertex
	SurfaceMesh::Halfedge nextEdge;
	Point p, p0, p1, d0, d1;
	Scalar theta, area;

	for (auto const& vertex : mesh.vertices())
	{
		theta = 0.0f;
		area = 0.0f;

		for (auto const& edge : mesh.halfedges(vertex))
		{
			nextEdge = mesh.next_halfedge(edge);

			v0 = mesh.from_vertex(nextEdge);
			v1 = mesh.to_vertex(nextEdge);

			p0 = vpoints[v0];
			p1 = vpoints[v1];
			p = vpoints[vertex];

			d0 = p0 - p;
			d1 = p1 - p;
			d0.normalize();
			d1.normalize();

			// Get the angle.
			theta += std::acos(d0.dot(d1));

			// Now get the area.
			area += (1 / 6.0f) * (d0.cross(d1)).norm();
		}

		vcurvature_K[vertex] = (2 * M_PI - theta) / area;
	}
	//end of computing gaussian curvature

	//compute voronoi area for each vertex
	for (auto const& face : mesh.faces()) {
		// Collect triangle vertices.
		auto vFit = mesh.vertices(face);
		v0 = *vFit;
		++vFit;
		v1 = *vFit;
		++vFit;
		v2 = *vFit;

		// Compute one third area.
		a = (vpoints[v1] - vpoints[v0]).cross(vpoints[v2] - vpoints[v0]).norm() / 6.0;

		// Distribute area to vertices
		varea[v0] += a;
		varea[v1] += a;
		varea[v2] += a;
	}
		//compute mean curvature for each vertex
		Scalar alpha, beta;
		Point laplace;

		for (const auto& vertex : mesh.vertices()) {
			laplace = Point(0, 0, 0);

			if (!mesh.is_boundary(vertex)) {
				for (const auto& neighbour : mesh.halfedges(vertex)) {
					nextEdge = mesh.next_halfedge(neighbour);

					v0 = mesh.from_vertex(nextEdge);
					v1 = mesh.to_vertex(nextEdge);

					p0 = vpoints[v0];
					p1 = vpoints[v1];
					p = vpoints[vertex];

					d0 = p - p0;
					d1 = p1 - p0;

					d0.normalize();
					d1.normalize();

					beta = std::acos(d0.dot(d1));

					d0 = p - p1;
					d1 = p0 - p1;

					d0.normalize();
					d1.normalize();

					alpha = std::acos(d0.dot(d1));

					Scalar cotanAlpha = 1.0f / std::tan(alpha);
					Scalar cotanBeta = 1.0f / std::tan(beta);

					laplace += (cotanAlpha + cotanBeta) * (p0 - p);
				}

				laplace /= 2.0 * varea[vertex];
			}

			vcurvature_H[vertex] = 0.5 * laplace.norm();
		}//end of computing mean curvatures

		//get k1 curvatures from mean and gaussian curvatures
		for (auto const& vertex : mesh.vertices())
			vcurvature_k1[vertex] = vcurvature_H[vertex] + sqrt((vcurvature_H[vertex] * vcurvature_H[vertex]) - vcurvature_K[vertex]);

		//get k2 curvatures from mean and gaussian curvatures
		for (auto const& vertex : mesh.vertices())
			vcurvature_k2[vertex] = vcurvature_H[vertex] - sqrt((vcurvature_H[vertex] * vcurvature_H[vertex]) - vcurvature_K[vertex]);

		//TODO:: get principal curvatures from k1, k2 curvatures.
	
}


/*deremine extremality coefficients e_i*/
void FeatureExtractor::BuildExtremeCoeffs()
{
	//TODO:
}

/// Initialization
void FeatureExtractor::init()
{
    // why? because face normals are needed to compute the initial quadrics
    mesh.update_face_normals();
	//first compute k1, k2 curvatures, which are the principal curvatures
	ComputeCurvatures();
	//Get Extremality coefficients from each points k1, k2 curvatures
	BuildExtremeCoeffs();

}

void FeatureExtractor::exec()
{

}
