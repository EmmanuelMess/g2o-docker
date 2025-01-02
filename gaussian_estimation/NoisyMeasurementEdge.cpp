#include "NoisyMeasurementEdge.hpp"

NoisyMeasurementEdge::NoisyMeasurementEdge() = default;

void NoisyMeasurementEdge::computeError() {
	const auto vertexInitial = dynamic_cast<const VertexBias*>(_vertices[0]);

	_error = _measurement - vertexInitial->estimate();
}

bool NoisyMeasurementEdge::read(std::istream& is) {
	for (int i = 0; i < _measurement.size() && is.good(); i++) {
		is >> _measurement(i);
	}
	readInformationMatrix(is);
	return is.good() || is.eof();
}

bool NoisyMeasurementEdge::write(std::ostream& os) const {
	const auto vector = measurement();
	for (int m = 0; m < vector.rows(); m++) {
		os << vector[m] << " ";
	}
	return writeInformationMatrix(os);
}