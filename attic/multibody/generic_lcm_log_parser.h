#pragma once

#include <string>
#include "drake/multibody/rigid_body.h"

namespace dairlib {
namespace multibody {

  template<typename T, typename U>
  void parseLcmLog(const RigidBodyTree<double>& tree,
      std::string file, std::string channel,
      Eigen::VectorXd* t, Eigen::MatrixXd* x,
      double duration = 1.0e-6);
}
}
