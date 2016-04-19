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
	//face properties
	fis_regular = mesh.add_face_property<bool>("f:is_regular");
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

//compute shape operators for every edge on the mesh
void FeatureExtractor::ComputeShapeOperators()
{
	std::cout << "computing shape operators..." << std::endl;
	//give every halfedge the norm of its corresponding face to compute dihedral angles for edge shape operators
	SurfaceMesh::Halfedge_around_face_circulator hfc, hfc_end;
	
	//compute voronoi area for each vertex and norm of each halfedge's corresponding face for dihedral angle calculation later.
	for (const auto& face : mesh.faces())
	{
		SurfaceMesh::Face_property<bool> is_reg = mesh.get_face_property<bool>("f:is_regular");
		is_reg[face] = true; //this will be used later for seeing if faces are regular or singular. For now we can ignore.
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

			//get dihedral angle theta
			Vec3 alpha = mesh.position(v0).cross(faceNorm[*hvc]);
			Vec3 beta = mesh.position(mesh.from_vertex(reciprocalEdge)).cross(faceNorm[reciprocalEdge]);
			float firstLength = sqrtf(powf(alpha.x(), 2) + powf(alpha.y(), 2) + powf(alpha.z(), 2));
			float secondLength = sqrtf(powf(beta.x(), 2) + powf(beta.y(), 2) + powf(beta.z(), 2));
			float theta = acosf(alpha.dot(beta) / (firstLength*secondLength));

			//compute direction of edge e. Orientation not important
			Vec3 p1 = mesh.position(mesh.from_vertex(*hvc));
			Vec3 p2 = mesh.position(mesh.to_vertex(*hvc));
			Vec3 e = (p2 - p1); 
			//compute mean curvature H_e for edge e
			float H_e = 2.0f * sqrtf(powf(e.x(), 2) + powf(e.y(), 2) + powf(e.z(), 2)) *(theta / 2.0f);
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
		} while (++hvc != hvc_end);

		vShapeOperator /= 2.0f;
		SurfaceMesh::Vertex_property<Mat3x3> vShapeOp = mesh.get_vertex_property<Mat3x3>("v:shape_operator");
		vShapeOp[vertex] = vShapeOperator;
	}
	std::cout << "done computing shape operators" << std::endl;
}

/*Get two highest absolute Eigenvalues from each vertices' shape operator. 
They are K_max and K_min, the discrete principal curvatures
Corresponding unit lenght eigenvectors are the principal directions*/
void FeatureExtractor::ComputeMaxMinCurvatures()
{
	std::cout << "computing k_max, k_min and their principal directions for each vertex.\nThis may take a minute..." << std::endl;
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
	//TODO: the following aren't done yet
	//discard singular triangles (May do this when rest of project is done) *UPDATE they are now marked as single though
	std::cout << "building extremality coefficients (e_i) for each vertex" << std::endl;
	SurfaceMesh::Face_around_vertex_circulator fvc, fvc_end;
	for (const auto& vertex : mesh.vertices()) //for every vertex...
	{
		fvc = mesh.faces(vertex);
		SurfaceMesh::Vertex_around_face_circulator vfc, vfc_end;
		fvc_end = fvc;
		Scalar e = 0.0;
		do //get area and derivative of mean curvature for triangle ::NOT SURE IF SHOULD BE MEAN CURVATURE
		{
			//first we gotta get the area of the triangle
			vfc = mesh.vertices(*fvc);
			vfc_end = vfc;
		
			Vec3 P = mesh.position(*vfc); ++vfc;
			Vec3 Q = mesh.position(*vfc); ++vfc;
			Vec3 R = mesh.position(*vfc);
			Vec3 PQ = Q - P;
			Vec3 PR = R - P;
			Vec3 crossVec = PQ.cross(PR); //triangle area is length of cross product
			Scalar areaT = sqrtf(powf(crossVec.x(), 2) + powf(crossVec.y(), 2) + powf(crossVec.z(), 2)) / 2.0f;

			//get mean curvature of triangle, computed as the average of each vertices k_1, k_2 curvatures.
			Vec3 GradKofTriangle(0, 0, 0);
			{
				vfc = mesh.vertices(*fvc);

				//		Scalar meanCurvature = 0.0;
				
				SurfaceMesh::Vertex_property<Scalar> k_min = mesh.get_vertex_property<Scalar>("v:curvature_kmin");
				SurfaceMesh::Vertex_property<Scalar> k_max = mesh.get_vertex_property<Scalar>("v:curvature_kmax");
				SurfaceMesh::Vertex_iterator Vi = *vfc;
				++vfc;
				SurfaceMesh::Vertex_iterator Vj = *vfc;
				++vfc;
				SurfaceMesh::Vertex_iterator Vk = *vfc;

				Vec3 gradB_i = ((mesh.position(*Vk) - (mesh.position(*Vj))) / (2 * areaT));
				Vec3 gradB_j = ((mesh.position(*Vk) - (mesh.position(*Vi))) / (2 * areaT));
				Vec3 gradB_k = ((mesh.position(*Vj) - (mesh.position(*Vi))) / (2 * areaT));
				GradKofTriangle = k_max[*Vi] * gradB_i + k_max[*Vj] * gradB_j + k_max[*Vk] * gradB_k;
			}

		//	meanCurvature /= 3.0f; //average it out/
		
			//Vec3 GradKofTriangle = 2 * meanCurvature*mesh.compute_face_normal(*fvc); //I believe this is correct from Laplace-Beltrami notes, but ASK FOR HELP
			SurfaceMesh::Vertex_property<Vec3> k_maxDir = mesh.get_vertex_property<Vec3>("v:direction_kmax");
			e += areaT * (GradKofTriangle.dot(k_maxDir[vertex]));
		} while (++fvc != fvc_end);
		SurfaceMesh::Vertex_property<Scalar> areaofP = mesh.get_vertex_property<Scalar>("v:area");
		e *= (1.0f / areaofP[vertex]);
	}
	std::cout << "done building extremality coefficients" << std::endl;
}

//flips signs of principal curvature directions k_min and k_max for vertices such that they are consistent for each triangle
//if this isn't possible, the triangle is 'singular', and we will regard it no more in our calculations
void FeatureExtractor::CorrectCurvatureSigns()
{
	std::cout << "making sure signs (+/-) of curvature directions are consistent for each triangle" << std::endl;
	SurfaceMesh::Vertex_around_face_circulator vfc, vfc_end;
	int singles = 0;
	
	for (const auto& face : mesh.faces())
	{
		vfc = mesh.vertices(face);
		vfc_end = vfc;
		SurfaceMesh::Vertex_iterator v0 = (*vfc); ++vfc;
		SurfaceMesh::Vertex_iterator v1 = (*vfc); ++vfc;
		SurfaceMesh::Vertex_iterator v2 = (*vfc);
		SurfaceMesh::Vertex_property<Vec3> k_iDir = mesh.get_vertex_property<Vec3>("v:direction_kmax");
		Vec3 k_iDirV0 = k_iDir[*v0];
		Vec3 k_iDirV1 = k_iDir[*v1];
		Vec3 k_iDirV2 = k_iDir[*v2];

		if (k_iDirV0.dot(k_iDirV1) > 0 && k_iDirV0.dot(k_iDirV2) > 0 && k_iDirV1.dot(k_iDirV2) > 0)
			continue;  //everything good! no signs need be flipped
		else
		{//find sign on vector(s) that needs flipping. If nothing works, mark face as singular.
			//just flip each vector one at a time and check for good fit
			Vec3 flippedVec = -1 * k_iDirV0;
			if (flippedVec.dot(k_iDirV1) > 0 && flippedVec.dot(k_iDirV2) > 0 && k_iDirV1.dot(k_iDirV2) > 0)
			{
			//	std::cout << "flipped" << std::endl;
				k_iDirV0 *= -1;
				continue;
			}
			flippedVec = -1 * k_iDirV1;
			if (k_iDirV0.dot(flippedVec) > 0 && k_iDirV0.dot(k_iDirV2) > 0 && flippedVec.dot(k_iDirV2) > 0)
			{
			//	std::cout << "flipped" << std::endl;
				k_iDirV1 *= -1;
				continue;
			}
			flippedVec = -1 * k_iDirV1;
			if (k_iDirV0.dot(k_iDirV1) > 0 && k_iDirV0.dot(flippedVec) > 0 && k_iDirV1.dot(flippedVec) > 0)
			{
			//	std::cout << "flipped" << std::endl;
				k_iDirV2 *= -1;
				continue;
			}
			else
			{//this face must forever bear the shame of being a single. May God have mercy on its soul.
				singles++;
				SurfaceMesh::Face_property<bool> is_reg = mesh.get_face_property<bool>("f:is_regular");
				is_reg[face] = false; //this will be used later for seeing if faces are regular or singular. For now we can ignore.
			}
		}
	}

	std::cout << "finished checking curvature direction signs. " << singles << " triangles are singles. This is " << (float(singles) / float(mesh.faces_size()))*100.0f
		<< " percent of the mesh's faces" << std::endl;
}

//now we can compute line segments that the feature lines!
void FeatureExtractor::ComputeFeatureLines()
{
	std::cout << "extracting feature lines. (not implemented yet)" << std::endl;
	int noCrossCount = 0;
	for (const auto& face : mesh.faces())
	{
		SurfaceMesh::Vertex_around_face_circulator vfc, vfc_end;
		vfc = mesh.vertices(face);
		vfc_end = vfc;
		SurfaceMesh::Vertex_iterator v0 = (*vfc); ++vfc;
		SurfaceMesh::Vertex_iterator v1 = (*vfc); ++vfc;
		SurfaceMesh::Vertex_iterator v2 = (*vfc);
		SurfaceMesh::Vertex_property<Vec3> k_iDir = mesh.get_vertex_property<Vec3>("v:direction_kmax");
		SurfaceMesh::Vertex_property<Scalar> e_i = mesh.get_vertex_property<Scalar>("v:extremality");
		SurfaceMesh::Vertex_property<Scalar> k_iMax = mesh.get_vertex_property<Scalar>("v:curvature_kmax");
		SurfaceMesh::Vertex_property<Scalar> k_iMin = mesh.get_vertex_property<Scalar>("v:curvature_kmin");

		Vec3 k_iDirV0 = k_iDir[*v0];
		Vec3 k_iDirV1 = k_iDir[*v1];
		Vec3 k_iDirV2 = k_iDir[*v2];

		Scalar e_0 = e_i[*v0];
		Scalar e_1 = e_i[*v1];
		Scalar e_2 = e_i[*v2];

		Vec3 PQ = mesh.position(*v1) - mesh.position(*v0);
		Vec3 PR = mesh.position(*v1) - mesh.position(*v2);
		Vec3 crossVec = PQ.cross(PR); //triangle area is length of cross product
		Scalar areaT = sqrtf(powf(crossVec.x(), 2) + powf(crossVec.y(), 2) + powf(crossVec.z(), 2)) / 2.0f;

			Vec3 gradB_i = ((mesh.position(*v2) - (mesh.position(*v1))) / (2 * areaT));
			Vec3 gradB_j = ((mesh.position(*v2) - (mesh.position(*v0))) / (2 * areaT));
			Vec3 gradB_k = ((mesh.position(*v1) - (mesh.position(*v0))) / (2 * areaT));

			Vec3 gradE_i = e_0 * gradB_i + e_1 * gradB_j + e_2 * gradB_k;
			
			//if either of these are true, no ridge line.
			if (gradE_i.dot((k_iDirV0 + k_iDirV1 + k_iDirV2)) > 0 ||
				(abs(k_iMax[*v0] + k_iMax[*v1] + k_iMax[*v2]) < abs(k_iMin[*v0] + k_iMin[*v1] + k_iMin[*v2]))) 
			{
				std::cout << "no crossing" << std::endl;
				noCrossCount += 1;
				continue;
			}
			
		} 
	std::cout << noCrossCount << std::endl;
}

//We need to process singular triangles to allow for stable computation of extremalities
void FeatureExtractor::ProcessSingularTriangles()
{
	//TODO::might not get done before demo.
}

/// Initialization
void FeatureExtractor::init()
{
    // why? because face normals are needed to compute the initial quadrics
    mesh.update_face_normals();
	mesh.update_vertex_normals();

	//get 3X3 tensor matrices for each vertices
	ComputeShapeOperators();
	//get min and max curvatures from the tensor matrix shape operators
	ComputeMaxMinCurvatures();
	BuildLinearFunctions();
	CorrectCurvatureSigns();
	ComputeFeatureLines();
	ProcessSingularTriangles();
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