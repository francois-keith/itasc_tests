#ifndef OROCOS_ITASC_SOLVER_WDLSPRIORVEL_TEST_COMPONENT_HPP
#define OROCOS_ITASC_SOLVER_WDLSPRIORVEL_TEST_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <sstream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

//TODO dynamically set the size of de and dq.
//TODO create several tests
//TODO provide a example with inequalities.

namespace iTaSC {
class Itasc_solver_wdlspriorvel_test : public RTT::TaskContext{
  public:
    Itasc_solver_wdlspriorvel_test(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
  private:
    unsigned int nq, nc;
    unsigned int priorityNo;

    // ...
    RTT::OutputPort<std::vector<int> > nc_priorities_port;
    std::vector<int> nc_priorities;

    // incoming result from solver.
    RTT::InputPort<Eigen::VectorXd> qdot_port;

    // data get from the config file.
    RTT::OutputPort<Eigen::VectorXd> qdot_expected_port;

    // Solution computed by the solver.
    Eigen::VectorXd qdot;

    // Solution expected.
    Eigen::VectorXd qdot_expected;
    RTT::OperationCaller<void(void)> solve;

    // Joint weigths
    RTT::OutputPort<Eigen::MatrixXd> Wq_port;
    Eigen::MatrixXd Wq;

    struct Priority
    {
      Priority();

      // data sent to the solver.
      RTT::OutputPort<Eigen::MatrixXd> A_port;
      RTT::OutputPort<Eigen::MatrixXd> Wy_port;
      RTT::OutputPort<Eigen::VectorXd> ydot_port;
      RTT::OutputPort<Eigen::VectorXd> ydot_max_port;

      KDL::Jacobian   A_kdl;
      Eigen::MatrixXd A;
      Eigen::MatrixXd Wy;
      Eigen::VectorXd ydot;
      Eigen::VectorXd ydot_max;

      // TODO it would be better to have a VectorXi, but the marshalling does not
      //  handle that ...
      RTT::OutputPort<Eigen::VectorXd> inequalities_port;
      Eigen::VectorXd inequalities;
    };

    std::vector<Priority*> priorities;
};
}
#endif
