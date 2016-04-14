#pragma once
#include <OpenGP/types.h>
using namespace OpenGP;

/// This class stores a quadric as a symmetrix 4x4 matrix used by the error quadric mesh decimation algorithms.
class Quadric {
    typedef Vec3 Normal;
    typedef Vec3 Point;
public:
    /// Zero constructor
    Quadric() {}

    /// constructs the quadric from the point and normal specifying a plane
    Quadric(const Normal& n, const Point& p) {
        /// TASK: initialize the quadric given a normal and a point on a supporting plane
		clear(); //make sure Q isn't full of garbage memory.
		float d_i = n.transpose()*(p);
		//get pbar and nbar so we have homogeneous coords
		Eigen::Vector4f pbar(p.x(), p.y(), p.z(), 1);
		Eigen::Vector4f nbar(n.x(), n.y(), n.z(), -d_i);
		//get Q_i, Kpp, whatever you want to call it.
		Q = nbar * nbar.transpose(); //or outer product of n by itself
		Q = Q.selfadjointView<Eigen::Upper>();
    }


    /// set all matric entries to zero
    void clear() {
        /// TASK: set the quadric to zero.
		Q.setZero();
    }


    /// add two quadrics
    Quadric operator+( const Quadric& _q ) {
        /// TASK: implement quadric add
		Q += _q.Q;
		return *this;
    }

    /// add given quadric to this quadric
    Quadric& operator+=( const Quadric& _q ) {
        /// TASK: implement quadric (self) add
		Q += _q.Q;
		return *this;
    }

    // evaluate quadric Q at position p by computing (p^T * Q * p)
    double evaluate(const Point& p) const {
        /// TASK: evaluate the quadratic form at point p
		Eigen::Vector4f pbar(p.x(), p.y(), p.z(), 1);//need homogeneous p
		return pbar.transpose()*Q*pbar;
    }

private:
    /// TASK: how to store the 4x4 symmetrix matrix?
    /// hint: see Eigen::SelfAdjointView
	Eigen::Matrix4f Q;
};
