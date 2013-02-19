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
    RTT::OutputPort<Eigen::MatrixXd> A_1_port;
    RTT::OutputPort<Eigen::MatrixXd> Wy_1_port;
    RTT::OutputPort<Eigen::VectorXd> ydot_1_port;
    RTT::OutputPort<Eigen::MatrixXd> Wq_port;
    RTT::InputPort<Eigen::VectorXd> qdot_port;

    KDL::Jacobian A_1_kdl;
    Eigen::MatrixXd A_1, Wy_1, Wq;
    Eigen::VectorXd ydot_1, qdot;
    RTT::OperationCaller<void(void)> solve;
};
}
#endif
