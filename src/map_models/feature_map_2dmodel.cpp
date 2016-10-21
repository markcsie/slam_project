#include "map_models/feature_map_2dmodel.h"

const std::string FeatureMap2dModel::TYPE = "Feature-based 2D Map Model";

FeatureMap2dModel::FeatureMap2dModel()
{
  type_ = "Feature-based 2D Map Model";
  dim_ = 2;
}

FeatureMap2dModel::FeatureMap2dModel(const FeatureMap2dModel& other)
{

}

FeatureMap2dModel::~FeatureMap2dModel()
{

}