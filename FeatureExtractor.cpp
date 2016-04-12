#include "FeatureExtractor.h"
#include <OpenGP/MLogger.h>
#include <math.h>

/// is the collapse of halfedge h allowed? (check for manifold, foldovers, etc...)




/// Initialization
void FeatureExtractor::init()
{
    // why? because face normals are needed to compute the initial quadrics
    mesh.update_face_normals();

}

void FeatureExtractor::exec()
{

}
