#ifndef INTERFACE_MAP_MODEL_H
#define INTERFACE_MAP_MODEL_H

#include <string>

class MapModelInterface
{
public:
  virtual const std::string &getType() const
  {
    return type_;
  };

  virtual const size_t &getDim() const
  {
    return dim_;
  };
  
protected:
  std::string type_;
  size_t dim_;
};

#endif /* INTERFACE_MAP_MODEL_H */

