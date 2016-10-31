#include "map_models/feature_map_2dmodel.h"

#include <iostream>

const std::string FeatureMap2dModel::TYPE = "Feature-based 2D Map Model";

FeatureMap2dModel::FeatureMap2dModel(const Eigen::VectorXd &size, const Eigen::VectorXd &corner)
{
  type_ = "Feature-based 2D Map Model";
  dim_ = 2;
  size_ = size;
  corner_ = corner;
}

FeatureMap2dModel::FeatureMap2dModel(const FeatureMap2dModel& other)
{

}

FeatureMap2dModel::~FeatureMap2dModel()
{
}