#include <string>
#include "attic/multibody/generic_lcm_log_parser.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

// #include "drake/lcm/drake_lcm_log.h"
// #include "drake/systems/analysis/simulator.h"
// #include "drake/systems/framework/diagram_builder.h"
// #include "drake/systems/lcm/lcm_log_playback_system.h"
// #include "drake/systems/lcm/lcm_subscriber_system.h"

// #include "systems/primitives/vector_aggregator.h"


// namespace dairlib {
// namespace multibody {

//   using std::string;
//   using Eigen::MatrixXd;
//   using Eigen::VectorXd;

//   template<typename T, typename U>
//   void parseLcmLog(const RigidBodyTree<double>& tree,
//       std::string file, std::string channel,
//       Eigen::VectorXd* t, Eigen::MatrixXd* x, double duration) {

//     using drake::systems::lcm::LcmSubscriberSystem;

//     drake::lcm::DrakeLcmLog r_log(file, false);

//     drake::systems::DiagramBuilder<double> builder;

//     builder.AddSystem<drake::systems::lcm::LcmLogPlaybackSystem>(&r_log);

//     auto output_sub = builder.AddSystem(
//         LcmSubscriberSystem::Make<T>(channel, &r_log));

//     auto output_receiver = builder.AddSystem<U>(tree);
//     builder.Connect(*output_sub, *output_receiver);

//     auto output_aggregator = builder.AddSystem<systems::VectorAggregator>(
//         output_receiver->get_output_port(0).size() - 1);
//     std::cout << output_receiver->get_output_port(0).size() - 1 << std::endl;

//     builder.Connect(*output_receiver, *output_aggregator);

//     auto diagram = builder.Build();

//     drake::systems::Simulator<double> sim(*diagram);

//     sim.StepTo(r_log.GetNextMessageTime() + duration);

//     *t = output_aggregator->BuildTimestampVector();

//     *x = output_aggregator->BuildMatrixFromVectors();
//   }

// } //namespace multibody
// } //namespace dairlib

int main() {
  Eigen::VectorXd t;
  Eigen::MatrixXd x;

  RigidBodyTree<double> tree;
  dairlib::buildCassieTree(tree);
  std::string channel = "CASSIE_STATE";
  std::string filename =
    "/home/nanda/Downloads/lcmlog-2019-06-27.00";

  // dairlib::multibody::parseLcmLog<dairlib::lcmt_robot_input,
  //   dairlib::systems::RobotInputReceiver>(tree, filename, channel, &t, &x, 36);

  dairlib::multibody::parseLcmLog<dairlib::lcmt_robot_output,
    dairlib::systems::RobotOutputReceiver>(tree, filename, channel, &t, &x, 36);

  std::cout << "*****t*****" << std::endl;
  std:: cout << t.rows() << " " << t.cols() << std::endl << std::endl;

  std::cout << "*****x*****" << std::endl;
  std:: cout << x.rows() << " " << x.cols() << std::endl << std::endl;

  return 0;
}
