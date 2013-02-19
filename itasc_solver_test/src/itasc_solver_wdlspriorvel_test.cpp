#include "itasc_solver_wdlspriorvel_test.hpp"
#include <ocl/Component.hpp>
#include <iostream>

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
    //this->properties()->addProperty("Wq", Wq);

    nc = nc_priorities[0];
    A_1.resize(nc, nq);
    Wy_1.resize(nc, nc);
    Wq.resize(nq, nq);
    ydot_1.resize(nc);
    qdot.resize(nq);
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
    Wq_port.write(Wq);
    //call solve()
    solve();
    //read from port
    qdot_port.read(qdot);
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
