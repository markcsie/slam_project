#include "interface_map_model.h"

#ifndef FEATURE_MAP_2DMODEL_H
#define FEATURE_MAP_2DMODEL_H

class FeatureMap2dModel : public MapModelInterface
{
public:
  FeatureMap2dModel(const Eigen::VectorXd &size, const Eigen::VectorXd &corner);
  FeatureMap2dModel(const FeatureMap2dModel& other);
  virtual ~FeatureMap2dModel();
  
  static const std::string TYPE;

};

#endif /* FEATURE_MAP_2DMODEL_H */

