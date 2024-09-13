#ifndef G2O_TARGET_TYPES_6D_HPP_
#define G2O_TARGET_TYPES_6D_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include <cassert>
#include "EigenTypes.hpp"

using namespace g2o;

struct State {
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
};

class VertexPositionVelocity3D : public g2o::BaseVertex<6, State> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexPositionVelocity3D() = default;

	void setToOriginImpl() override {
		_estimate.position.setZero();
		_estimate.velocity.setZero();
	}

	void oplusImpl(const double* update) override {
		for (int k = 0; k < 3; k++) {
			_estimate.position[k] += update[k];
			_estimate.velocity[k] += update[k+3];
		}
	}

	bool read(std::istream& is) override {
		for (int i = 0; i < _estimate.position.size() && is.good(); i++) {
			is >> _estimate.position(i);
		}
		for (int i = 0; i < _estimate.velocity.size() && is.good(); i++) {
			is >> _estimate.velocity(i);
		}
		return is.good() || is.eof();
	}

	bool write(std::ostream& os) const override {
		auto vector = estimate();
		for (int m = 0; m < vector.position.rows(); m++) {
		  os << vector.position[m] << " ";
		}
		for (int m = 0; m < vector.velocity.rows(); m++) {
			os << vector.velocity[m] << " ";
		}
		return os.good();
	}

};

// The odometry which links pairs of nodes together
class TargetOdometry3DEdge : public g2o::BaseBinaryEdge<6, Eigen::Vector3d, VertexPositionVelocity3D, VertexPositionVelocity3D> {
public:
	TargetOdometry3DEdge() : _dt(0) {}

	TargetOdometry3DEdge(double dt, double noiseSigma) {
		_dt = dt;

		const double q = noiseSigma * noiseSigma;
		const double dt2 = dt * dt;

		// Process noise covariance matrix; this assumes an "impulse"
		// noise model; we add a small stabilising term on the diagonal to make it
		// invertible
		const double a = dt2 * dt2 * q / 4 + 1e-4;
		const double b = dt * dt2 * q / 2;
		const double c = dt2 * q + 1e-4;
		const double d = dt * dt2 * q / 2;

		Matrix6d Q = Matrix6d::Zero();
		Q << a, 0, 0, b, 0, 0,
				 0, a, 0, 0, b, 0,
				 0, 0, a, 0, 0, b,
				 d, 0, 0, c, 0, 0,
				 0, d, 0, 0, c, 0,
				 0, 0, d, 0, 0, c;

		setInformation(Q.inverse());
	}

	/** set the estimate of the to vertex, based on the estimate of the from
	 * vertex in the edge. */
	void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to) override {
		assert(from.size() == 1);
		const VertexPositionVelocity3D* vi = dynamic_cast<const VertexPositionVelocity3D*>(*from.begin());
		auto vj = dynamic_cast<VertexPositionVelocity3D*>(to);
		const auto oldState = vi->estimate();

		const auto oldVelocity = oldState.velocity;
		const auto measuredAcceleration = _measurement;
		const auto estimatedPositionChange = _dt * oldVelocity + 0.5 * _dt * _dt * measuredAcceleration;
		const auto estimatedVelocityChange = _dt * measuredAcceleration;

		State newState = {
			.position = oldState.position + estimatedPositionChange,
			.velocity = oldState.velocity + estimatedVelocityChange,
		};

		vj->setEstimate(newState);
	}

	/** override in your class if it's not possible to initialize the vertices in
	 * certain combinations */
	double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to) override {
		// only works on sequential vertices
		const VertexPositionVelocity3D* vi = dynamic_cast<const VertexPositionVelocity3D*>(*from.begin());
		if (to->id() - vi->id() == 1) {
			return 1.0;
		} else {
			return -1.0;
		}
	}

	void computeError() override {
		const auto vertexInitial = dynamic_cast<const VertexPositionVelocity3D*>(_vertices[0]);
		const auto vertexFinal =dynamic_cast<const VertexPositionVelocity3D*>(_vertices[1]);

		const auto initialPosition = vertexInitial->estimate().position;
		const auto initialVelocity = vertexInitial->estimate().velocity;
		const auto finalPosition = vertexFinal->estimate().position;
		const auto finalVelocity = vertexFinal->estimate().velocity;
		const auto measuredAcceleration = _measurement;

		const auto estimatedPosition = initialPosition + _dt * initialVelocity + (0.5 * _dt * _dt) * measuredAcceleration;
    const auto positionError = estimatedPosition - finalPosition;

		const auto estimatedVelocity = initialVelocity + _dt * measuredAcceleration;
		const auto velocityError = estimatedVelocity - finalVelocity;

		_error.head(3) = positionError;
		_error.tail(3) = velocityError;
	}

	bool read(std::istream& is) override {
		for (int i = 0; i < _measurement.size() && is.good(); i++) {
			is >> _measurement(i);
		}
		readInformationMatrix(is);
		return is.good() || is.eof();
	}

	bool write(std::ostream& os) const override {
		auto vector = measurement();
		for (int m = 0; m < vector.rows(); m++) {
			os << vector[m] << " ";
		}
		return writeInformationMatrix(os);
	}

private:
	double _dt;
};

// The GPS
class GPSObservationEdgePositionVelocity3D : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPositionVelocity3D> {
public:
	GPSObservationEdgePositionVelocity3D() = default;

	GPSObservationEdgePositionVelocity3D(const Eigen::Vector3d& measurement, double noiseSigma) {
		_measurement = measurement;
		setInformation(Eigen::Matrix3d::Identity() / (noiseSigma * noiseSigma));
	}

	void computeError() override {
		const auto v = dynamic_cast<const VertexPositionVelocity3D*>(_vertices[0]);
		const auto lastPosition = v->estimate().position;

		_error = lastPosition - _measurement;
	}

	bool read(std::istream& is) override {
		for (int i = 0; i < _measurement.size() && is.good(); i++) {
			is >> _measurement(i);
		}
		readInformationMatrix(is);
		return is.good() || is.eof();
	}

	bool write(std::ostream& os) const override {
		auto vector = measurement();
		for (int m = 0; m < vector.rows(); m++) {
			os << vector[m] << " ";
		}
		return writeInformationMatrix(os);
	}
};

#endif  //  G2O_TARGET_TYPES_6D_HPP_