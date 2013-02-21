#include "itasc_solver_wdlspriorvel_test.hpp"
#include <ocl/Component.hpp>
#include <iostream>

namespace
{
  bool compare (const Eigen::VectorXd & v1, const Eigen::VectorXd & v2, bool tol = 1e-9)
  {
    if (v1.size() != v2.size())
      return false;
    for (unsigned i=0; i<v1.size(); ++i)
    {
      if (fabs(v1[i] - v2[i]) > tol)
        return false;
    }
    return true;
  }
}

// It is possible to define the port in the configureHook method.
//  BUT it may now become impossible to see them.
// The properties have to be defined before calling configure hook.
namespace iTaSC {
using namespace RTT;
using namespace KDL;

Itasc_solver_wdlspriorvel_test::Itasc_solver_wdlspriorvel_test(std::string const& name) : TaskContext(name),
nq(7),
priorityNo(1),
nc_priorities(std::vector<int>(1,6)),
A_1_kdl(Jacobian(7))
{
    this->ports()->addPort("Wq",Wq_port).doc("weights on robot joints");
    this->ports()->addPort("qdot",qdot_port).doc("desired robot joint            velocities");
    this->addPort("nc_priorities",nc_priorities_port).doc("Port with vector of       number of constraints per priority.");
    this->ports()->addPort("A_1", A_1_port).doc("general jacobian");
    this->ports()->addPort("ydot_1", ydot_1_port).doc("ydot");
    this->ports()->addPort("Wy_1", Wy_1_port).doc("Wy");

    this->properties()->addProperty("nq", nq);
    this->properties()->addProperty("priorityNo", priorityNo);
    this->properties()->addProperty("nc_priorities", nc_priorities);
    this->properties()->addProperty("A_1", A_1_kdl);
    //this->properties()->addProperty("Wy_1", Wy_1);
    this->properties()->addProperty("ydot_1", ydot_1);

    this->ports()->addPort("ydot_max_1", ydot_max_1_port).doc("ydot_max");
    this->properties()->addProperty("ydot_max_1", ydot_max_1);

    //
    this->ports()->addPort("qdot_expected",qdot_expected_port).doc("expected qdot");
    this->properties()->addProperty("qdot_expected", qdot_expected);

    this->ports()->addPort("inequalities_1", inequalities_1_port).doc("inequalities indexes");
    this->properties()->addProperty("inequalities_1", inequalities_1);

    //this->properties()->addProperty("Wq", Wq);

    nc = nc_priorities[0];
    A_1.resize(nc, nq);
    Wy_1.resize(nc, nc);
    Wq.resize(nq, nq);
    ydot_1.resize(nc);
    ydot_max_1.resize(0);


    qdot.resize(nq);
    qdot_expected.resize(nq);
    inequalities_1.resize(0);

    Wy_1.setIdentity();
    Wq.setIdentity();
    std::cout << "Itasc_solver_wdlspriorvel_test constructed !" <<std::endl;
}

bool Itasc_solver_wdlspriorvel_test::configureHook(){
    nc = nc_priorities[0];
    Wy_1.resize(nc, nc);
    Wq.resize(nq, nq);
    Wy_1.setIdentity();
    Wq.setIdentity();

    A_1=A_1_kdl.data;
    TaskContext* solver_ptr = getPeer("Solver");
  //set priority number and nq
    Attribute<unsigned int> nq_att = solver_ptr->provides()->getAttribute("nq");
    Attribute<unsigned int> priorityNo_att = solver_ptr->provides()->getAttribute("priorityNo");
    nq_att.set(nq);
    priorityNo_att.set(priorityNo);

  //set nc_priorities on port
    nc_priorities_port.write(nc_priorities);
  //get solve method (but don't call it yet
    solve = solver_ptr->getOperation("solve"); 

  std::cout << "Itasc_solver_wdlspriorvel_test configured !" <<std::endl;
  return true;
}

bool Itasc_solver_wdlspriorvel_test::startHook(){
  std::cout << "Itasc_solver_wdlspriorvel_test started !" <<std::endl;
  return true;
}

void Itasc_solver_wdlspriorvel_test::updateHook(){
    //put dummy data on port
    A_1_port.write(A_1);
    Wy_1_port.write(Wy_1);
    ydot_1_port.write(ydot_1);
    ydot_max_1_port.write(ydot_max_1);

    //assertion: we only use inequalities if the solver can handle it

    //TODO: the solver doesn't have inequalityProvisions, but there are inequalities present
    //TaskContext* solver_ptr = getPeer("Solver");
    //if((!solver_ptr->inequalityProvisions) && inequalities_1.size() != 0)
    //  log(Error) << "[[TestSolver] error: The problem has inequalities, but solver can't handle inequalities" << endlog();
    //else
      inequalities_1_port.write(inequalities_1);

    Wq_port.write(Wq);

    //call solve()
    solve();

    //read from port
    if( qdot_port.read(qdot) != RTT::NoData)
    {
        // Validate the received solution.
        if( (compare(qdot, qdot_expected, 1e-9)) )
        {
          std::cout << "Success ! " << std::endl;
          // Todo: interrupt properly
          //  ie not std::exit(0);
        }
        else
        {
          std::cerr.precision(12);
          std::cerr << "Fuu ! " << std::endl;
          std::cerr << " qdot " << qdot.transpose() << std::endl;
          std::cerr << " qdot_expected " << qdot_expected.transpose() << std::endl;
          std::cerr << " diff = " << (qdot_expected - qdot).transpose() << std::endl;
    //      std::exit(-1);
        }
    }
}

void Itasc_solver_wdlspriorvel_test::stopHook() {
  std::cout << "Itasc_solver_wdlspriorvel_test executes stopping !" <<std::endl;
}

void Itasc_solver_wdlspriorvel_test::cleanupHook() {
  std::cout << "Itasc_solver_wdlspriorvel_test cleaning up !" <<std::endl;
}

}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Itasc_solver_wdlspriorvel_test)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(iTaSC::Itasc_solver_wdlspriorvel_test);
