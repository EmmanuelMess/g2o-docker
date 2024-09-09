#ifndef G2O_TARGET_TYPES_6D_HPP_
#define G2O_TARGET_TYPES_6D_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include <cassert>
#include "EigenTypes.hpp"

using namespace g2o;

class VertexPositionVelocity3D : public g2o::BaseVertex<6, Vector6d> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexPositionVelocity3D() = default;

	void setToOriginImpl() override { _estimate.setZero(); }

	void oplusImpl(const double* update) override {
		for (int k = 0; k < 6; k++) _estimate[k] += update[k];
	}

	bool read(std::istream& is) override {
		for (int i = 0; i < _estimate.size() && is.good(); i++) {
			is >> _estimate(i);
		}
		return is.good() || is.eof();
	}

	bool write(std::ostream& os) const override {
		auto vector = estimate();
		for (int m = 0; m < vector.rows(); m++) {
		  os << vector[m] << " ";
		}
		return os.good();
	}

};

// The odometry which links pairs of nodes together
class TargetOdometry3DEdge
	: public g2o::BaseBinaryEdge<6, Eigen::Vector3d, VertexPositionVelocity3D,
		VertexPositionVelocity3D> {
public:
	TargetOdometry3DEdge() = default;

	TargetOdometry3DEdge(double dt, double noiseSigma) {
		_dt = dt;

		double q = noiseSigma * noiseSigma;
		double dt2 = dt * dt;

		// Process noise covariance matrix; this assumes an "impulse"
		// noise model; we add a small stabilising term on the diagonal to make it
		// invertible
		Matrix6d Q = Matrix6d::Zero();
		Q(0, 0) = Q(1, 1) = Q(2, 2) = dt2 * dt2 * q / 4 + 1e-4;
		Q(0, 3) = Q(1, 4) = Q(2, 5) = dt * dt2 * q / 2;
		Q(3, 3) = Q(4, 4) = Q(5, 5) = dt2 * q + 1e-4;
		Q(3, 0) = Q(4, 1) = Q(5, 2) = dt * dt2 * q / 2;

		setInformation(Q.inverse());
	}

	/** set the estimate of the to vertex, based on the estimate of the from
	 * vertex in the edge. */
	void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to) override {
		assert(from.size() == 1);
		const VertexPositionVelocity3D* vi = dynamic_cast<const VertexPositionVelocity3D*>(*from.begin());
		auto vj = dynamic_cast<VertexPositionVelocity3D*>(to);
		Vector6d viEst = vi->estimate();
		Vector6d vjEst = viEst;

		for (int m = 0; m < 3; m++) {
			vjEst[m] += _dt * (vjEst[m + 3] + 0.5 * _dt * _measurement[m]);
		}

		for (int m = 0; m < 3; m++) {
			vjEst[m + 3] += _dt * _measurement[m];
		}
		vj->setEstimate(vjEst);
	}

	/** override in your class if it's not possible to initialize the vertices in
	 * certain combinations */
	double initialEstimatePossible(
		const g2o::OptimizableGraph::VertexSet& from,
		g2o::OptimizableGraph::Vertex* to) override {
		// only works on sequential vertices
		const VertexPositionVelocity3D* vi =
			dynamic_cast<const VertexPositionVelocity3D*>(*from.begin());
		return (to->id() - vi->id() == 1) ? 1.0 : -1.0;
	}

	void computeError() override {
		const auto vi = dynamic_cast<const VertexPositionVelocity3D*>(_vertices[0]);
		const auto vj =dynamic_cast<const VertexPositionVelocity3D*>(_vertices[1]);

		for (int k = 0; k < 3; k++) {
			_error[k] = vi->estimate()[k] +
			            _dt * (vi->estimate()[k + 3] + 0.5 * _dt * _measurement[k]) -
			            vj->estimate()[k];
		}
		for (int k = 3; k < 6; k++) {
			_error[k] =
				vi->estimate()[k] + _dt * _measurement[k - 3] - vj->estimate()[k];
		}
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
class GPSObservationEdgePositionVelocity3D
	: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPositionVelocity3D> {
public:
	GPSObservationEdgePositionVelocity3D() = default;

	GPSObservationEdgePositionVelocity3D(const Eigen::Vector3d& measurement, double noiseSigma) {
		_measurement = measurement;
		setInformation(Eigen::Matrix3d::Identity() / (noiseSigma * noiseSigma));
	}

	void computeError() override {
		const auto v = dynamic_cast<const VertexPositionVelocity3D*>(_vertices[0]);
		for (int k = 0; k < 3; k++) {
			_error[k] = v->estimate()[k] - _measurement[k];
		}
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