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
    std::vector<int> nc_priorities;
    RTT::OutputPort<std::vector<int> > nc_priorities_port;

    // data sent to the solver.
    RTT::OutputPort<Eigen::MatrixXd> A_1_port;
    RTT::OutputPort<Eigen::MatrixXd> Wy_1_port;
    RTT::OutputPort<Eigen::VectorXd> ydot_1_port;
    RTT::OutputPort<Eigen::VectorXd> ydot_max_1_port;
    RTT::OutputPort<Eigen::VectorXd> inequalities_1_port;

    RTT::OutputPort<Eigen::MatrixXd> Wq_port;

    // incoming result from solver.
    RTT::InputPort<Eigen::VectorXd> qdot_port;

    // data get from the config file.
    RTT::OutputPort<Eigen::VectorXd> qdot_expected_port;

    KDL::Jacobian A_1_kdl;
    Eigen::MatrixXd A_1, Wy_1, Wq;
    Eigen::VectorXd ydot_1;
    Eigen::VectorXd ydot_max_1;
    Eigen::VectorXd inequalities_1;

    Eigen::VectorXd qdot;
    Eigen::VectorXd qdot_expected;
    RTT::OperationCaller<void(void)> solve;
};
}
#endif
