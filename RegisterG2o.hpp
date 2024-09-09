#ifndef G2OTEST_REGISTERG2O_HPP
#define G2OTEST_REGISTERG2O_HPP

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

#include "targetTypes6D.hpp"

namespace g2o {
	G2O_REGISTER_TYPE_GROUP(slam3d);

	G2O_REGISTER_TYPE(VERTEX_VELOCITY, VertexPositionVelocity3D);
	G2O_REGISTER_TYPE(EDGE_ODOMETRY, TargetOdometry3DEdge);
	G2O_REGISTER_TYPE(EDGE_GPS_POSITION, GPSObservationEdgePositionVelocity3D);
}

#endif //G2OTEST_REGISTERG2O_HPP
