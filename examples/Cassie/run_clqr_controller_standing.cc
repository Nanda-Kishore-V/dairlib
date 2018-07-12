#include <memory>

#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/clqr_controller.h"
#include "systems/framework/output_vector.h"
#include "systems/primitives/subvector_pass_through.h"
#include "multibody/solve_multibody_constraints.h"
#include "examples/Cassie/cassie_utils.h"

using std::cout;
using std::endl;
using std::vector;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using drake::VectorX;
using drake::MatrixX;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::BasicVector;
using drake::systems::ConstantVectorSource;
using drake::systems::Multiplexer;

using dairlib::multibody::SolveTreeConstraints;
using dairlib::multibody::CheckTreeConstraints;
using dairlib::multibody::SolveFixedPointConstraints;
using dairlib::multibody::CheckFixedPointConstraints;
using dairlib::multibody::SolveFixedPointConstraintsApproximate;
using dairlib::multibody::SolveTreeAndFixedPointConstraints;
using dairlib::multibody::CheckTreeAndFixedPointConstraints;
using dairlib::multibody::SolveFixedPointFeasibilityConstraints;
using dairlib::systems::AffineParams;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::OutputVector;


namespace dairlib{


// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.7, "The static coefficient of friction");
DEFINE_double(ud, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 1e-2,
              "The characteristic scale of contact patch (m)");
DEFINE_string(simulation_type, "compliant", "The type of simulation to use: "
              "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3, "The step size to use for "
              "'simulation_type=timestepping' (ignored for "
              "'simulation_type=compliant'");


//Class to serve as a connecer between the OutputVector type input port of the clqr controller and a BasicVector port through which the plant states are sent
class InfoConnector: public LeafSystem<double> {

  public:

    InfoConnector(int num_positions, int num_velocities, int num_efforts):
      num_states_(num_positions + num_velocities), num_efforts_(num_efforts)
    {
      this->DeclareVectorInputPort(BasicVector<double>(
            num_positions + num_velocities + num_efforts + 3 + 1));
      this->DeclareVectorOutputPort(OutputVector<double>(
            num_positions, num_velocities, num_efforts), &dairlib::InfoConnector::CopyOut);
    }

    const int num_states_;
    const int num_efforts_;

  private:

    void CopyOut(const Context<double>& context, OutputVector<double>* output) const
    {
      const auto info = this->EvalVectorInput(context, 0);
      const VectorX<double> info_vec = info->get_value();
      output->SetState(info_vec.head(num_states_));
      output->set_timestamp(0);
    }

};

//Class to serve as a pass through port that just prints the value of the input port. Useful for debugging
class DebugPassThrough: public LeafSystem<double> {

  public:
    DebugPassThrough(int size, bool debug_flag = true): size_(size), debug_flag_(debug_flag) {

    this->DeclareVectorInputPort(BasicVector<double>(size_));
    this->DeclareVectorOutputPort(BasicVector<double>(size_), &dairlib::DebugPassThrough::CopyOut);
  }

  const int size_;
  bool debug_flag_;

  private:
    void CopyOut(const Context<double>& context, BasicVector<double>* output) const {

      const auto input = this->EvalVectorInput(context, 0);
      const VectorX<double> input_vector = input->get_value();


      if(debug_flag_) {
        cout << input_vector.transpose() << endl;
        cout << "------------------------------------------------------------------------------------------" << endl;
      }
      output->SetFromVector(input_vector);
    }
};

//Function to take the state of a floating base model and extract the state for the fixed base model by removing the 
//positions and velocities corresponding to the base
VectorXd ExtractFixedStateFromFloat(VectorXd x_float, int num_positions_float, int num_velocities_float, int num_extra_positions, int num_extra_velocities)
{
  const int num_positions_fixed = num_positions_float - num_extra_positions;
  const int num_velocities_fixed = num_velocities_float - num_extra_velocities;
  const int fixed_size = num_positions_fixed + num_velocities_fixed;

  VectorXd x_fixed(fixed_size);
  x_fixed.head(num_positions_fixed) = x_float.segment(6, num_positions_fixed);
  x_fixed.tail(num_velocities_fixed) = x_float.segment(num_positions_float + 6, num_velocities_fixed);

  return x_fixed;
}

VectorXd ComputeUAnalytical(const RigidBodyTree<double>& tree, VectorXd x) {

  MatrixXd B = tree.B;
  auto k_cache = tree.doKinematics(x.head(tree.get_num_positions()), x.tail(tree.get_num_velocities()));
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd C = tree.dynamicsBiasTerm(k_cache, no_external_wrenches, true);
  VectorXd u = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(C);

  return u;

}




int do_main(int argc, char* argv[]) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  drake::lcm::DrakeLcm lcm;
  std::unique_ptr<RigidBodyTree<double>> tree_sim = makeFloatingBaseCassieTreePointer();
  std::unique_ptr<RigidBodyTree<double>> tree_model = makeFixedBaseCassieTreePointer();

  //const double toe_contact_distance = 0.2;
  //VectorXd toe_left_ground = VectorXd::Zero(3);
  //VectorXd toe_right_ground = VectorXd::Zero(3);
  //toe_left_ground(1) = 0.3;
  //toe_right_ground(1) = -0.3;

  //tree->addDistanceConstraint(0, toe_left_ground, 18, VectorXd::Zero(3), toe_contact_distance);
  //tree->addDistanceConstraint(0, toe_right_ground, 20, VectorXd::Zero(3), toe_contact_distance);


  //Floating base adds 6 additional states for the base position and orientation
  const int num_total_positions = tree_sim->get_num_positions();
  const int num_total_velocities = tree_sim->get_num_velocities();
  const int num_total_states = num_total_positions + num_total_velocities;
  const int num_positions = tree_model->get_num_positions();
  const int num_velocities = tree_model->get_num_velocities();
  const int num_states = num_positions + num_velocities;
  const int num_efforts = tree_model->get_num_actuators();
  const int num_constraints = tree_model->getNumPositionConstraints();
  
  cout << "Number of actuators: " << num_efforts << endl;
  cout << "Number of generalized coordinates: " << num_positions << endl;
  cout << "Number of generalized velocities: " << num_velocities << endl;
  cout << "Number of tree constraints: " << num_constraints << endl;

  const double terrain_size = 4;
  const double terrain_depth = 0.05;

  drake::multibody::AddFlatTerrainToWorld(tree_sim.get(), terrain_size, terrain_depth);
  drake::multibody::AddFlatTerrainToWorld(tree_model.get(), terrain_size, terrain_depth);
  
  cout << "---------------------------------------------------------------------" << endl;

  for(int i=0; i<tree_sim->get_num_bodies(); i++)
  {
    cout << tree_sim->get_body(i).get_name() << " " << i << endl;
  }


  cout << "---------------------------------------------------------------------" << endl;
  
  drake::systems::DiagramBuilder<double> builder;
  
  auto plant_sim = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree_sim));
  auto plant_model = make_unique<RigidBodyPlant<double>>(std::move(tree_model));

  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant_sim->set_default_compliant_material(default_material);
  plant_model->set_default_compliant_material(default_material);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant_sim->set_contact_model_parameters(model_parameters);
  plant_model->set_contact_model_parameters(model_parameters);


  // Adding the visualizer to the diagram
  drake::systems::DrakeVisualizer& visualizer_publisher =
      *builder.template AddSystem<drake::systems::DrakeVisualizer>(
          plant_sim->get_rigid_body_tree(), &lcm);
  visualizer_publisher.set_name("visualizer_publisher");
  //builder.Connect(plant->state_output_port(),
                          //visualizer_publisher.get_input_port(0));

  auto debug_pass_through = builder.AddSystem<DebugPassThrough>(num_total_states, false);
  builder.Connect(plant_sim->state_output_port(), debug_pass_through->get_input_port(0));
  builder.Connect(debug_pass_through->get_output_port(0), visualizer_publisher.get_input_port(0));


  VectorXd x0 = VectorXd::Zero(num_total_states);
  std::map<std::string, int>  map_sim = plant_sim->get_rigid_body_tree().computePositionNameToIndexMap();
  std::map<std::string, int>  map_model = plant_model->get_rigid_body_tree().computePositionNameToIndexMap();

  for(auto elem: map_sim)
  {
      cout << elem.first << " " << elem.second << endl;
  }


  x0(map_sim.at("base_x")) = 0.0;
  x0(map_sim.at("base_y")) = 0.0;
  x0(map_sim.at("base_z")) = 1.129;

  //x0(map.at("hip_roll_left")) = 0.15;
  //x0(map.at("hip_yaw_left")) = 0.2;
  x0(map_sim.at("hip_pitch_left")) = .269;
  x0(map_sim.at("hip_pitch_right")) = .269;
  // x0(map.at("achilles_hip_pitch_left")) = -.44;
  // x0(map.at("achilles_hip_pitch_right")) = -.44;
  // x0(map.at("achilles_heel_pitch_left")) = -.105;
  // x0(map.at("achilles_heel_pitch_right")) = -.105;
  x0(map_sim.at("knee_left")) = -.644;
  x0(map_sim.at("knee_right")) = -.644;
  x0(map_sim.at("ankle_joint_left")) = .792;
  x0(map_sim.at("ankle_joint_right")) = .792;
  
  // x0(map.at("toe_crank_left")) = -90.0*M_PI/180.0;
  // x0(map.at("toe_crank_right")) = -90.0*M_PI/180.0;
  
  // x0(map.at("plantar_crank_pitch_left")) = 90.0*M_PI/180.0;
  // x0(map.at("plantar_crank_pitch_right")) = 90.0*M_PI/180.0;
  
  x0(map_sim.at("toe_left")) = -80*M_PI/180.0;
  x0(map_sim.at("toe_right")) = -80*M_PI/180.0;

  std::vector<int> fixed_joints;

  //fixed_joints.push_back(map.at("hip_roll_left"));
  //fixed_joints.push_back(map.at("hip_yaw_left"));
  fixed_joints.push_back(map_model.at("hip_pitch_left"));
  fixed_joints.push_back(map_model.at("hip_pitch_right"));
  fixed_joints.push_back(map_model.at("knee_left"));
  fixed_joints.push_back(map_model.at("knee_right"));
  fixed_joints.push_back(map_model.at("toe_left"));
  fixed_joints.push_back(map_model.at("toe_right"));


  cout << "Works till here" << endl;
  
  VectorXd x_start = x0;
  
  //const RigidBodyTree<double>& tree_p = plant->get_rigid_body_tree();
  //auto k_cache = tree_p.doKinematics(x_start.head(num_total_positions));
  //auto k_cache_element = k_cache.get_element(18);
  //Eigen::Transform<double, 3, Eigen::Isometry> tr = k_cache_element.transform_to_world;
  //cout << tr.matrix() << endl;
  //cout << k_cache_element.transform_to_world << endl;


  VectorXd q_start = SolveTreeConstraints(plant_model->get_rigid_body_tree(), x_start.segment(6, num_positions), fixed_joints);
  x_start.segment(6, num_positions) = q_start;

  cout << "x_start: " << x_start.transpose() << endl;

  VectorXd x_init = ExtractFixedStateFromFloat(x_start, num_total_positions, num_total_velocities, 6, 6);
  cout << "x_init: " << x_init.transpose() << endl;
  VectorXd q_init = x_start.head(num_total_positions);
  VectorXd v_init = x_start.tail(num_total_velocities);
  VectorXd u_init = VectorXd::Zero(num_efforts);
  
  //Parameter matrices for LQR
  MatrixXd Q = MatrixXd::Identity(num_states - 2*num_constraints, num_states - 2*num_constraints);
  //Q corresponding to the positions
  MatrixXd Q_p = MatrixXd::Identity(num_states/2 - num_constraints, num_states/2 - num_constraints)*1000.0;
  //Q corresponding to the velocities
  MatrixXd Q_v = MatrixXd::Identity(num_states/2 - num_constraints, num_states/2 - num_constraints)*10.0;
  Q.block(0, 0, Q_p.rows(), Q_p.cols()) = Q_p;
  Q.block(num_states/2 - num_constraints, num_states/2 - num_constraints, Q_v.rows(), Q_v.cols()) = Q_v;
  MatrixXd R = MatrixXd::Identity(num_efforts, num_efforts)*100.0;

  vector<VectorXd> sol_tfp = SolveTreeAndFixedPointConstraints(plant_model.get(), x_init, ComputeUAnalytical(plant_model->get_rigid_body_tree(), x_init));
  VectorXd q_sol = sol_tfp.at(0);
  VectorXd v_sol = sol_tfp.at(1);
  VectorXd u_sol = sol_tfp.at(2);
  VectorXd x_sol(num_states);
  x_sol << q_sol, v_sol;

  //x_start.segment(6, num_positions

  cout << "x_sol: " << x_sol.transpose() << endl;
  cout << "x_sol2: " << sol_tfp2.at(0).transpose() << " " << sol_tfp2.at(1).transpose() << endl;

  x_start.segment(6, num_positions) = q_sol;
  x_start.segment(num_total_positions + 6, num_velocities) = v_sol;

  //VectorXd u_a = ComputeUAnalytical(plant_model->get_rigid_body_tree(), sol_fp.at(0));

  //vector<VectorXd> sol_tpfp = SolveTreeAndFixedPointConstraints(plant_model, x_init, u_init);
  //vector<VectorXd> sol_fpa = SolveFixedPointConstraintsApproximate(plant_model.get(), x_init, u_init);
  //VectorXd u_sol = sol_fpa.at(0);
  //cout << u_sol.transpose() << endl;

  //cout << "Solved Tree Position and Fixed Point constraints" << endl;

  //VectorXd q_sol = sol_tpfp.at(0);
  //VectorXd v_sol = sol_tpfp.at(1);
  //VectorXd u_sol = sol_tpfp.at(2);
  //VectorXd x_sol(num_states);
  //x_sol << q_sol, v_sol;



  //Building the controller
  //auto clqr_controller = builder.AddSystem<systems::ClqrController>(plant, x_sol, u_sol, NUM_POSITIONS, NUM_VELOCITIES, NUM_EFFORTS, Q, R);
  //VectorXd K_vec = clqr_controller->GetKVec();
  //VectorXd C = u_sol; 
  //VectorXd x_desired = x_sol;
  //cout << "K: " << K_vec.transpose() << endl;
  //cout << "C: " << C.transpose() << endl;
  //cout << "xdes: " << x_desired.transpose() << endl;

  //vector<int> input_info_sizes{NUM_STATES, NUM_EFFORTS, 3, 1};
  //vector<int> input_params_sizes{NUM_STATES*NUM_EFFORTS, NUM_EFFORTS, NUM_STATES, 1};

  //auto info_connector = builder.AddSystem<InfoConnector>(NUM_POSITIONS, NUM_VELOCITIES, NUM_EFFORTS);
  //auto multiplexer_info = builder.AddSystem<Multiplexer<double>>(input_info_sizes);

  //auto constant_zero_source_efforts = builder.AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(NUM_EFFORTS));
  //auto constant_zero_source_imu = builder.AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(3));
  //auto constant_zero_source_timestamp = builder.AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(1));

  //VectorXd params_vec(NUM_STATES*NUM_EFFORTS + NUM_EFFORTS + NUM_STATES);
  //params_vec << K_vec, C, x_desired;
  //AffineParams params(NUM_STATES, NUM_EFFORTS);
  //params.SetDataVector(params_vec);
  //auto constant_params_source = builder.AddSystem<ConstantVectorSource<double>>(params);

  //auto control_output = builder.AddSystem<SubvectorPassThrough<double>>(
  //        (clqr_controller->get_output_port(0)).size(), 0, (clqr_controller->get_output_port(0)).size() - 1);

  //builder.Connect(plant->state_output_port(), multiplexer_info->get_input_port(0));
  //builder.Connect(constant_zero_source_efforts->get_output_port(), multiplexer_info->get_input_port(1));
  //builder.Connect(constant_zero_source_imu->get_output_port(), multiplexer_info->get_input_port(2));
  //builder.Connect(constant_zero_source_timestamp->get_output_port(), multiplexer_info->get_input_port(3));
  //builder.Connect(multiplexer_info->get_output_port(0), info_connector->get_input_port(0));
  //builder.Connect(info_connector->get_output_port(0), clqr_controller->get_input_port_info());
  //builder.Connect(constant_params_source->get_output_port(), clqr_controller->get_input_port_params());
  //builder.Connect(clqr_controller->get_output_port(0), control_output->get_input_port());
  //builder.Connect(control_output->get_output_port(), plant->actuator_command_input_port()); 

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context = diagram->GetMutableSubsystemContext(*plant_sim, &simulator.get_mutable_context());
  
  drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state(); 
  state.SetFromVector(x_start);
  
  auto zero_input = Eigen::MatrixXd::Zero(num_efforts,1);
  context.FixInputPort(0, zero_input);
  
  //simulator.set_publish_every_time_step(false);
  //simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  
  lcm.StartReceiveThread();
  
  simulator.StepTo(std::numeric_limits<double>::infinity());
  return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
