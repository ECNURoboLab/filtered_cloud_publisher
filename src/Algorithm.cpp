#include "filtered_cloud_publisher/Algorithm.hpp"

namespace filtered_cloud_publisher {

Algorithm::Algorithm()
    : average_(0.0),
      nMeasurements_(0)
{
}

Algorithm::~Algorithm()
{
}

void Algorithm::addData(const double data)
{
  average_ = (nMeasurements_ * average_ + data) / (nMeasurements_ + 1);
  nMeasurements_++;
}

double Algorithm::getAverage() const
{
  return average_;
}

} /* namespace */
