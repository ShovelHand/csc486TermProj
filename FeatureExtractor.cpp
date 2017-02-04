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
	vdirection_kmax = mesh.add_vertex_property<OpenGP::Vec3>("v:direction_kmax");
	vdirection_kmin = mesh.add_vertex_property<OpenGP::Vec3>("v:direction_kmin");
	vcurvature_kmax = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_kmax");
	vcurvature_kmin = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_kmin");
	vShapeOperator = mesh.add_vertex_property<OpenGP::Mat3x3>("v:shape_operator");
	vextremality = mesh.add_vertex_property<OpenGP::Scalar>("v:extremality");
	//edge properties
	eShapeOperator = mesh.add_edge_property<OpenGP::Mat3x3>("e:shape_operator");
	hface_norm = mesh.add_halfedge_property<OpenGP::Vec3>("h:face_norm");
	hTraversed_this_time = mesh.add_halfedge_property<bool>("h:hTraversed_this_time");
	//face properties
	fis_regular = mesh.add_face_property<bool>("f:is_regular");
}

FeatureExtractor::~FeatureExtractor()
{
	mesh.remove_vertex_property(varea);
	mesh.remove_vertex_property(vcurvature_kmax);
	mesh.remove_vertex_property(vcurvature_kmin);

	//TODO:: remember to remove vertex properties, etc that you've haphazardly added
	mesh.garbage_collection();
}

//vertex shape operators are an average of edge shape operators, so I 
//encapsulated that computation here for code readability
OpenGP::Mat3x3 FeatureExtractor::ComputeEdgeShapeOperators(SurfaceMesh::Vertex_iterator vertex){
	Mat3x3 vertexShapeOperator;
	vertexShapeOperator.Zero();

	//assign each halfedge its face's surface norm for calculating dihedral angles
	SurfaceMesh::Face_around_vertex_circulator fc, fc_end;
		fc = mesh.faces(*vertex);
		fc_end = fc;

		do {
			SurfaceMesh::Halfedge_around_face_circulator hc, hc_end;
			hc = mesh.halfedges(*fc);
			hc_end = hc;

			do{
				hface_norm[*hc] = mesh.compute_face_normal(*fc);
			} while (++hc != hc_end);

		} while (++fc != fc_end);

	//iterate over each edge from the vertex
	SurfaceMesh::Vertex v0, v1;
	SurfaceMesh::Halfedge_around_vertex_circulator hvc, hvc_end;
	hvc = mesh.halfedges(*vertex);
	hvc_end = hvc;
	//std::cout << *vertex << std::endl;
	hvc = mesh.halfedges(*vertex);
	hvc_end = hvc;
	
	//this is so we can mark halfEdges as traversed so we only compute the shape operator once per edge, not both halfs
	//But we should make sure they all start as untraversed
	
	do{
		hTraversed_this_time[*hvc] = false;
	} while (++hvc != hvc_end);

	do
	{
		
		v0 = mesh.from_vertex(*hvc);
		v1 = mesh.to_vertex(*hvc);
		SurfaceMesh::Halfedge thisEdge = mesh.find_halfedge(v0, v1);
		if (!hTraversed_this_time[thisEdge])//only once per edge, not both half-edges
		{
			SurfaceMesh::Halfedge reciprocalEdge = mesh.find_halfedge(v1, v0);
			//because we got the face normal for each halfedge above we should now be able to compute the dihedral angle,
			//where cos(t) = n[1] dot n[2]
			float theta = acosf(hface_norm[thisEdge].dot(hface_norm[reciprocalEdge]));
			//now get mean curvature of edge, where H[e] = 2|e|cos(theta[e]/2)
			Vec3 e = vpoints[v1] - vpoints[v0];
			float H = 2.0f * sqrtf(pow(e.x(), 2) + pow(e.y(), 2) + pow(e.z(), 2)) * cosf(theta / 2.0f);

			//and now we can compute the edge's shape operator
			Mat3x3 edgeShapeOperator = H * (e.normalized().cross(hface_norm[thisEdge]))*(e.normalized().cross(hface_norm[thisEdge])).transpose();
			//calculate <N[p], N[e]>
			Vec3 Np = mesh.compute_vertex_normal(*vertex);
			float innerProduct = Np.x() * hface_norm[thisEdge].x() + Np.y() * hface_norm[thisEdge].y() + Np.z() * hface_norm[thisEdge].z();
			vertexShapeOperator += innerProduct * edgeShapeOperator;
			hTraversed_this_time[thisEdge] = true; // hTraversed_this_time[reciprocalEdge] = true;
			std::cout << *vertex << std::endl;
		}
	} while (++hvc != hvc_end);

	return vertexShapeOperator * 0.5f;
}


//compute shape operators for every vertex on the mesh
void FeatureExtractor::ComputeVertexShapeOperators()
{
	std::cout << "computing shape operators..." << std::endl;

	// loop over all vertices
	for (const auto& vertex : mesh.vertices())
	{
		// initialize circulators
	//	vc = mesh.vertices(*vit);
	//	vc_end = vc;
		vShapeOperator[vertex] = ComputeEdgeShapeOperators(vertex);
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

		//do a lame bubble sort to get two highest values
		bool swapped = true;
		while (swapped)
		{
		
			for (int i = 0; i < eigenPairs.size() - 1; ++i)
			{
				if (abs(eigenPairs[i].first) > abs(eigenPairs[i + 1].first))
				{//swap
					std::pair<float, Vec3> swap = eigenPairs[i];
					eigenPairs[i] = eigenPairs[i + 1];
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
		vk_min[vertex] = k_min *(3.0f / varea[vertex]);
		vk_max[vertex] = k_max *(3.0f / varea[vertex]);
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
	//discard singular triangles (May do this when rest of project is done) *UPDATE they are now marked as single 
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
		SurfaceMesh::Vertex_property<Scalar> e_i = mesh.get_vertex_property<Scalar>("v:extremality");
		e_i[vertex] = e;
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
	std::cout << "extracting feature lines." << std::endl;
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
			//	std::cout << "no crossing" << std::endl;
				noCrossCount += 1;
				continue;
			}
			
			//otherwise, lets find the point of that zero crossing for two edges on the triangle.
			//e_i's must be different signs for this to make sense
			Vec3 p1(-1, -1, -1);
			Vec3 p2(-1,-1,-1);
			if ((e_0 < 0 && e_1 > 0) || (e_1 > 0 && e_1 < 0))
			{
				if (p1 == Vec3(-1, -1, -1))
					//p1 = (mesh.position(*v1) + mesh.position(*v0)/2.0f);
					p1 = ComputeRidgePoint(v1, v0);
				else
					p2 = (ComputeRidgePoint(v1, v0));
			}
			if ((e_0 < 0 && e_2 > 0) || (e_0 > 0 && e_2 < 0))
			{
				if (p1 == Vec3(-1, -1, -1))
					p1 = ComputeRidgePoint(v2, v0);
				else
					p2 = ComputeRidgePoint(v2, v0);
			}
			if ((e_1 < 0 && e_2 > 0) || (e_1 > 0 && e_2 < 0))
			{
				if (p1 == Vec3(-1, -1, -1))
					p1 = ComputeRidgePoint(v2, v1);
				else
					p2 = ComputeRidgePoint(v2, v1);
			}

			if (p1 != Vec3(-1, -1, -1) && p2 != Vec3(-1, -1, -1))
			{
				ridgeLinePoints.push_back(p1);
				ridgeLinePoints.push_back(p2);
			}
		/*	else
			{
				std::cout << "no crossing in this face" << std::endl;
			}*/
			
		/*	for (std::vector<Vec3>::iterator itr = ridgeLinePoints.begin(); itr != ridgeLinePoints.end(); ++itr)
			{
				std::cout << (*itr) << "\n\n" << std::endl;
			}*/
		} 
	std::cout << noCrossCount << std::endl;
}

//this is a hacky and improper way of computing the positions for ridge line segment ends
//right now it just returns the position of the vertex with the extremality of highest absolute value.
Vec3 FeatureExtractor::ComputeRidgePoint(SurfaceMesh::Vertex_iterator v1, SurfaceMesh::Vertex_iterator v2)
{
	Vec3 dir = mesh.position(*v2) - mesh.position(*v1); dir.normalize(); //normal vector in direction of edge
	SurfaceMesh::Vertex_property<Scalar> e_i = mesh.get_vertex_property<Scalar>("v:extremality");
//	Scalar t;
	Vec3 ret;
	if (abs(e_i[*v1]) > abs(e_i[*v2]))
	{
	//	t = e_i[*v2] / e_i[*v1];
		ret = mesh.position(*v1);
	}
	else if (abs(e_i[*v2]) > abs(e_i[*v1]))
	{
//		t = e_i[*v1] / e_i[*v2];
		ret = mesh.position(*v2);
	}

	return ret;
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
	//mesh.update_vertex_normals();

	//get 3X3 tensor matrices for each vertices
	ComputeVertexShapeOperators();
	//get min and max curvatures from the tensor matrix shape operators
	ComputeMaxMinCurvatures();
	BuildLinearFunctions();
	CorrectCurvatureSigns();
	ComputeFeatureLines();
//	ProcessSingularTriangles();
}
