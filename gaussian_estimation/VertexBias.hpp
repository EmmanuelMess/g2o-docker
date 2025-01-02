#ifndef VERTEXBIAS_HPP
#define VERTEXBIAS_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

class VertexBias : public g2o::BaseVertex<2, Eigen::Vector2d> {
public:
	VertexBias() = default;

	void setToOriginImpl() override;

	void oplusImpl(const double* update) override;

	bool read(std::istream& is) override;

	bool write(std::ostream& os) const override;
};



#endif //VERTEXBIAS_HPP
