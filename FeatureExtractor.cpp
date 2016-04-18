#include "FeatureExtractor.h"
#include <OpenGP/MLogger.h>
#include <math.h>

FeatureExtractor::FeatureExtractor(SurfaceMesh& mesh) : mesh(mesh)
{
//face properties	
	fnormals = mesh.face_property<Normal>("f:normal");
	//vertex properties
	varea = mesh.add_vertex_property<OpenGP::Scalar>("v:area");
	vpoints = mesh.vertex_property<Point>("v:point");
	vquality = mesh.vertex_property<float>("v:quality");
	vcurvature_K = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_K");
	vcurvature_H = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_H");
	vdirection_kmax = mesh.add_vertex_property<OpenGP::Vec3>("v:direction_kmax");
	vdirection_kmin = mesh.add_vertex_property<OpenGP::Vec3>("v:direction_kmin");
	vcurvature_kmax = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_kmax");
	vcurvature_kmin = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_kmin");
	vcurvature_principal = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_principal");
	vShapeOperator = mesh.add_vertex_property<OpenGP::Mat3x3>("v:shape_operator");
	vextremality = mesh.add_vertex_property<OpenGP::Scalar>("v:extremality");
	//edge properties
	ecotan = mesh.add_edge_property<OpenGP::Scalar>("e:cotan");
	eShapeOperator = mesh.add_edge_property<OpenGP::Mat3x3>("e:shape_operator");
	hface_norm = mesh.add_halfedge_property<OpenGP::Vec3>("h:face_norm");
}

FeatureExtractor::~FeatureExtractor()
{
	mesh.remove_vertex_property(varea);
	//mesh.remove_vertex_property(ecotan);
	mesh.remove_vertex_property(vcurvature_K);
	mesh.remove_vertex_property(vcurvature_H);
	mesh.remove_vertex_property(vcurvature_kmax);
	mesh.remove_vertex_property(vcurvature_kmin);
	mesh.remove_vertex_property(vcurvature_principal);
	//TODO:: remember to remove vertex properties, etc that you've haphazardly added
	mesh.garbage_collection();
}
//
//void FeatureExtractor::ComputeCurvatures()
//{
//	//compute mean curvatures
//	for (auto const& vertex : mesh.vertices()) {
//		varea[vertex] = 0.0;
//	}
//	SurfaceMesh::Vertex_around_face_circulator vFit;
//	SurfaceMesh::Vertex v0, v1, v2;
//	Scalar a;
//
//	//compute gaussian curvature for each vertex
//	SurfaceMesh::Halfedge nextEdge;
//	Point p, p0, p1, d0, d1;
//	Scalar theta, area;
//
//	for (auto const& vertex : mesh.vertices())
//	{
//		theta = 0.0f;
//		area = 0.0f;
//
//		for (auto const& edge : mesh.halfedges(vertex))
//		{
//			nextEdge = mesh.next_halfedge(edge);
//
//			v0 = mesh.from_vertex(nextEdge);
//			v1 = mesh.to_vertex(nextEdge);
//
//			p0 = vpoints[v0];
//			p1 = vpoints[v1];
//			p = vpoints[vertex];
//
//			d0 = p0 - p;
//			d1 = p1 - p;
//			d0.normalize();
//			d1.normalize();
//
//			// Get the angle.
//			theta += std::acos(d0.dot(d1));
//
//			// Now get the area.
//			area += (1 / 6.0f) * (d0.cross(d1)).norm();
//		}
//
//		vcurvature_K[vertex] = (2 * M_PI - theta) / area;
//	}
//	//end of computing gaussian curvature
//
//	//compute voronoi area for each vertex
//	for (auto const& face : mesh.faces()) {
//		// Collect triangle vertices.
//		auto vFit = mesh.vertices(face);
//		v0 = *vFit;
//		++vFit;
//		v1 = *vFit;
//		++vFit;
//		v2 = *vFit;
//
//		// Compute one third area.
//		a = (vpoints[v1] - vpoints[v0]).cross(vpoints[v2] - vpoints[v0]).norm() / 6.0;
//
//		// Distribute area to vertices
//		varea[v0] += a;
//		varea[v1] += a;
//		varea[v2] += a;
//	}
//		//compute mean curvature for each vertex
//		Scalar alpha, beta;
//		Point laplace;
//
//		for (const auto& vertex : mesh.vertices()) {
//			laplace = Point(0, 0, 0);
//
//			if (!mesh.is_boundary(vertex)) {
//				for (const auto& neighbour : mesh.halfedges(vertex)) {
//					nextEdge = mesh.next_halfedge(neighbour);
//
//					v0 = mesh.from_vertex(nextEdge);
//					v1 = mesh.to_vertex(nextEdge);
//
//					p0 = vpoints[v0];
//					p1 = vpoints[v1];
//					p = vpoints[vertex];
//
//					d0 = p - p0;
//					d1 = p1 - p0;
//
//					d0.normalize();
//					d1.normalize();
//
//					beta = std::acos(d0.dot(d1));
//
//					d0 = p - p1;
//					d1 = p0 - p1;
//
//					d0.normalize();
//					d1.normalize();
//
//					alpha = std::acos(d0.dot(d1));
//
//					Scalar cotanAlpha = 1.0f / std::tan(alpha);
//					Scalar cotanBeta = 1.0f / std::tan(beta);
//
//					laplace += (cotanAlpha + cotanBeta) * (p0 - p);
//				}
//
//				laplace /= 2.0 * varea[vertex];
//			}
//
//			vcurvature_H[vertex] = 0.5 * laplace.norm();
//		}//end of computing mean curvatures
//
//		//get k1 curvatures from mean and gaussian curvatures
//		for (auto const& vertex : mesh.vertices())
//			vcurvature_k1[vertex] = vcurvature_H[vertex] + sqrt((vcurvature_H[vertex] * vcurvature_H[vertex]) - vcurvature_K[vertex]);
//
//		//get k2 curvatures from mean and gaussian curvatures
//		for (auto const& vertex : mesh.vertices())
//			vcurvature_k2[vertex] = vcurvature_H[vertex] - sqrt((vcurvature_H[vertex] * vcurvature_H[vertex]) - vcurvature_K[vertex]);
//
//		create_colours(vcurvature_k1);
//
//		//determine which curvature, k1, or k2, is the principal curvature
//		//the lines of largest principal curvatures are the crest lines
//		for (auto const& vertex : mesh.vertices())
//		{
//			if (abs(vcurvature_k1[vertex]) > abs(vcurvature_k2[vertex]))
//			{
//				vcurvature_principal[vertex] = vcurvature_k1[vertex];
//			}
//			else if (abs(vcurvature_k2[vertex]) > abs(vcurvature_k1[vertex]))
//			{
//				vcurvature_principal[vertex] = vcurvature_k2[vertex];
//			}
//		}
//}

//compute shape operators for every edge on the mesh
void FeatureExtractor::ComputeShapeOperators()
{
	//give every halfedge the norm of its corresponding face to compute dihedral angles for edge shape operators
	SurfaceMesh::Halfedge_around_face_circulator hfc, hfc_end;
	
	//compute voronoi area for each vertex and norm of each halfedge's corresponding face for dihedral angle calculation later.
	for (const auto& face : mesh.faces())
	{
		//initialize iterators
		hfc = mesh.halfedges(face);
		hfc_end = hfc;
		
		Scalar a;
		SurfaceMesh::Vertex v0, v1, v2;
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

		do
		{
			hface_norm[*hfc] = mesh.compute_face_normal(face);
		} while (++hfc != hfc_end);
	}

	SurfaceMesh::Halfedge_around_vertex_circulator hvc, hvc_end;
	//now we can get dihedral angle of each edge, and compute edge based shape operators.
	for (const auto& vertex : mesh.vertices())
	{
		SurfaceMesh::Vertex v0, v1;
		//initialize iterators 
		hvc = mesh.halfedges(vertex);
		hvc_end = hvc;
		do
		{
			v0 = mesh.from_vertex(*hvc);
			v1 = mesh.to_vertex(*hvc);
			SurfaceMesh::Halfedge reciprocalEdge = mesh.find_halfedge(v1, v0);
			//get face normal associated with both halfedges to compute dihedral angle
			SurfaceMesh::Halfedge_property<Vec3> faceNorm = mesh.get_halfedge_property<Vec3>("h:face_norm");
			//ASK ABOUT EIGEN OPERATORS FOR GETTING VECTOR LENGTH
			float firstLength = sqrtf(powf(faceNorm[*hvc].x(), 2)+ powf(faceNorm[*hvc].y(), 2)+ powf(faceNorm[*hvc].z(), 2));
			float secondLength = sqrtf(powf(faceNorm[reciprocalEdge].x(), 2) + powf(faceNorm[reciprocalEdge].y(), 2) + powf(faceNorm[reciprocalEdge].z(), 2));
			float theta = acos((faceNorm[*hvc].dot(faceNorm[reciprocalEdge]))/(firstLength*secondLength));
			//compute direction of edge e. Orientation not important
			Vec3 p1 = mesh.position(mesh.from_vertex(*hvc));
			Vec3 p2 = mesh.position(mesh.to_vertex(*hvc));
			Vec3 e = (p2 - p1); 
			//compute mean curvature H_e for edge e
			float H_e = 2.0f * sqrtf(powf(e.x(), 2) + powf(e.y(), 2) + powf(e.z(), 2)) *(theta / 2.0f);//NOT SURE THIS IS RIGHT, ASK ANDREA. PARTICULARLY NOT SURE IF LENGHT OF E IS WHAT IS DESIRED
			e.normalize();
			//get N_e, the average of the surface normals on either side of the edge
			Vec3 N_e = (faceNorm[*hvc] + faceNorm[reciprocalEdge]) / (firstLength + secondLength);
			//now we can compute the shape operator for the edge
			Mat3x3 edgeShapeOperator = H_e * (e.cross(N_e))*(e.cross(N_e)).transpose();
			SurfaceMesh::Edge thisEdge = mesh.find_edge(v0, v1);
			eShapeOperator[thisEdge] = edgeShapeOperator;
		} while (++hvc != hvc_end);
	}

	//now we'll use the edge shape operators to make the vertex shape operators
	for (const auto& vertex : mesh.vertices())
	{
		Mat3x3 vShapeOperator; 
		vShapeOperator.Zero();
		Vec3 N_p = mesh.compute_vertex_normal(vertex);
		SurfaceMesh::Vertex v0, v1;
		hvc = mesh.halfedges(vertex);
		hvc_end = hvc;
		float avgDiv = 0.0f;
		
		do
		{
			v0 = mesh.from_vertex(*hvc);
			v1 = mesh.to_vertex(*hvc);
			SurfaceMesh::Halfedge reciprocalEdge = mesh.find_halfedge(v1, v0);
			//get face normal associated with both halfedges to compute dihedral angle
			SurfaceMesh::Halfedge_property<Vec3> faceNorm = mesh.get_halfedge_property<Vec3>("h:face_norm");
			//ASK ABOUT EIGEN OPERATORS FOR GETTING VECTOR LENGTH
			float firstLength = sqrtf(powf(faceNorm[*hvc].x(), 2) + powf(faceNorm[*hvc].y(), 2) + powf(faceNorm[*hvc].z(), 2));
			float secondLength = sqrtf(powf(faceNorm[reciprocalEdge].x(), 2) + powf(faceNorm[reciprocalEdge].y(), 2) + powf(faceNorm[reciprocalEdge].z(), 2));
			Vec3 N_e = (faceNorm[*hvc] + faceNorm[reciprocalEdge]) / (firstLength + secondLength);
			//get edge shape operator for each edge from vertex
			SurfaceMesh::Edge thisEdge = mesh.find_edge(v0, v1); 
			SurfaceMesh::Edge_property<Mat3x3> eShapeOperator = mesh.get_edge_property<Mat3x3>("e:shape_operator");
			vShapeOperator += (N_p.dot(N_e))* eShapeOperator[thisEdge];
			avgDiv += 1;
		} while (++hvc != hvc_end);
		
		assert(avgDiv > 0);//ASK ABOUT THIS. PAPER SHOWS 1/2 IN FRONT OF SIGMA, BUT SAYS IT'S AN AVERAGE
		vShapeOperator /= avgDiv;
		SurfaceMesh::Vertex_property<Mat3x3> vShapeOp = mesh.get_vertex_property<Mat3x3>("v:shape_operator");
		vShapeOp[vertex] = vShapeOperator;
	}

}

/*Get two highest absolute Eigenvalues from each vertices' shape operator. 
They are K_max and K_min, the discrete principal curvatures
Corresponding unit lenght eigenvectors are the principal directions*/
void FeatureExtractor::ComputeMaxMinCurvatures()
{
	std::cout << "computing k_max, k_min and their principal directions for each vertex.\nThis'll take a minute..." << std::endl;
	SurfaceMesh::Vertex_property<Vec3> vk_minDir = mesh.get_vertex_property<Vec3>("v:direction_kmin");
	SurfaceMesh::Vertex_property<Vec3> vk_maxDir = mesh.get_vertex_property<Vec3>("v:direction_kmax");
	SurfaceMesh::Vertex_property<Scalar> vk_min = mesh.get_vertex_property<Scalar>("v:curvature_kmin");
	SurfaceMesh::Vertex_property<Scalar> vk_max = mesh.get_vertex_property<Scalar>("v:curvature_kmax");
	SurfaceMesh::Vertex_property<Mat3x3> vShapeOp = mesh.get_vertex_property<Mat3x3>("v:shape_operator");

	int i = 0;

	//We'll need to solve eigenvalues/vectors for each vertice's shape operator
	for (const auto& vertex : mesh.vertices())
	{
		i++;
		Eigen::EigenSolver<Eigen::Matrix3f> es(vShapeOp[vertex]);//thank goodness for this, now let's get our two largest absolute values
		Scalar k_max, k_min;
		Vec3 k_maxDir, k_minDir;

		std::vector<std::pair<Scalar, Vec3> > eigenPairs;
		std::pair<Scalar, Vec3> eigenValVec;
		eigenValVec.first = es.eigenvalues()(0, 0).real(); eigenValVec.second = es.eigenvectors().col(0).real();
		eigenPairs.push_back(eigenValVec);

		eigenValVec.first = es.eigenvalues()(1, 0).real(); eigenValVec.second = es.eigenvectors().col(0).real();
		eigenPairs.push_back(eigenValVec);

		eigenValVec.first = es.eigenvalues()(2, 0).real(); eigenValVec.second = es.eigenvectors().col(2).real();
		eigenPairs.push_back(eigenValVec);

		bool swapped = true;
		while (swapped)
		{
			//do a lame bubble sort to get two highest values
			for (int i = 0; i < eigenPairs.size() - 1; ++i)
			{
				if (abs(eigenPairs[i].first) > abs(eigenPairs[i + 1].first))
				{//swap
					std::pair<float, Vec3> swap = eigenPairs[i];
					eigenPairs[i] = eigenPairs[i+1];
					eigenPairs[i + 1] = swap;
					swapped = true;
				}
				else
				{
					swapped = false;
				}
			}
		}
		//now sort these in increasing order so that we can pop last two for our values
		k_max = eigenPairs.back().first; k_maxDir = eigenPairs.back().second;
		eigenPairs.pop_back();
		k_min = eigenPairs.back().first; k_minDir = eigenPairs.back().second;
		//we'll also scale the curvatures here since "by construction, [they're] integrated quantities" (see paper)
		vk_min[vertex] = k_min * (3.0f / varea[vertex]);
		vk_max[vertex] = k_max * (3.0f / varea[vertex]);
		vk_minDir[vertex] = k_minDir.normalized();
		vk_maxDir[vertex] = k_maxDir.normalized();

		if (i % 1000 == 0)
		{
			std::cout << "processed " << i << " of "<< mesh.n_vertices() << " vertices" << std::endl;
		}
	}

	std::cout << "done finding principal curvatures and their directions" << std::endl;
}

/*deremine build piecewise linear functions by triangle*/
void FeatureExtractor::BuildLinearFunctions()
{
	//TODO:
	//remember to have good choices for sign of k_i vectors
	//discard singular triangles (May do this when rest of project is done)
	std::cout << "building extremality coefficients (e_i) for each vertex" << std::endl;
	SurfaceMesh::Face_around_vertex_circulator fvc, fvc_end;
	for (const auto& vertex : mesh.vertices())
	{
		fvc = mesh.faces(vertex);
		fvc_end = fvc;

		do
		{

		} while (++fvc != fvc_end);
		
	}

	std::cout << "done building extremality coefficients" << std::endl;
}

/// Initialization
void FeatureExtractor::init()
{
    // why? because face normals are needed to compute the initial quadrics
    mesh.update_face_normals();

	//get 3X3 tensor matrices for each vertices
	ComputeShapeOperators();
	//get min and max curvatures from the tensor matrix shape operators
	ComputeMaxMinCurvatures();
	BuildLinearFunctions();
}

void FeatureExtractor::exec()
{

}
void FeatureExtractor::create_colours(OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> prop) {
	using namespace OpenGP;

	// Determine min and max meanCature (remove outliers)
	// Use relative property value are 1D texture coordinate in range [0, 1].
	std::vector<Scalar> values;
	for (const auto& vertex : mesh.vertices()) {
		values.push_back(prop[vertex]);
	}

	// Sort array.
	std::sort(values.begin(), values.end());

	// Discard lower and upper 2%.
	unsigned int n = values.size() - 1;
	unsigned int i = n / 50;
	Scalar minProp = values[i];
	Scalar maxProp = values[n - 1 - i];

	for (const auto& vertex : mesh.vertices()) {
		vquality[vertex] = (prop[vertex] - minProp) / (maxProp - minProp);
	}
}