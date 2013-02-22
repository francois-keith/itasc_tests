#include "itasc_solver_wdlspriorvel_test.hpp"
#include <ocl/Component.hpp>
#include <iostream>

namespace
{
  bool compare (const Eigen::VectorXd & v1, const Eigen::VectorXd & v2, double tol)
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

Itasc_solver_wdlspriorvel_test::Itasc_solver_wdlspriorvel_test(std::string const& name)
: TaskContext(name),
nq(7),
priorityNo(2),
nc_priorities(),
testDone(false)
{
    nc_priorities.resize(priorityNo,0);
//    nc_priorities[0] = 2;
//    nc_priorities[1] = 6;

    this->ports()->addPort("qdot",qdot_port).doc("desired robot joint            velocities");
    this->addPort("nc_priorities",nc_priorities_port).doc("Port with vector of       number of constraints per priority.");
    this->provides()->addAttribute("priorityNo", priorityNo); //"Number of priorities involved. (default = 1)"


    this->properties()->addProperty("nq", nq);
    this->properties()->addProperty("priorityNo", priorityNo);
    this->properties()->addProperty("nc_priorities", nc_priorities);

    //
    this->ports()->addPort("qdot_expected",qdot_expected_port).doc("expected qdot");
    this->properties()->addProperty("qdot_expected", qdot_expected);

    //
    this->ports()->addPort("Wq", Wq_port).doc("weights on robot joints");
    Wq = Eigen::MatrixXd::Identity(nq, nq);
    //this->properties()->addProperty("Wq", Wq);

    //
    qdot.resize(nq);
    qdot_expected.resize(nq);

    std::stringstream ssName;
    std::string pname;
    priorities.resize(priorityNo);

    // TODO Priority:creator
    for (unsigned int i=0;i<priorityNo;i++)
    {
        priorities[i] = new Priority();
        int nc = nc_priorities[i];

        ssName.clear();
        ssName << "A_" << i+1;
        ssName >> pname;
        this->ports()->addPort(pname, priorities[i]->A_port).doc("general jacobian");
        this->properties()->addProperty(pname, priorities[i]->A_kdl);

        ssName.clear();
        ssName << "Wy_" << i+1;
        ssName >> pname;
        this->ports()->addPort(pname, priorities[i]->Wy_port).doc("Wy");
        //this->properties()->addProperty("Wy_1", Wy_1);

        ssName.clear();
        ssName << "ydot_" << i+1;
        ssName >> pname;
        this->ports()->addPort(pname, priorities[i]->ydot_port).doc("ydot");
        this->properties()->addProperty(pname, priorities[i]->ydot);

        ssName.clear();
        ssName << "ydot_max_" << i+1;
        ssName >> pname;
        this->ports()->addPort(pname, priorities[i]->ydot_max_port).doc("ydot_max");
        this->properties()->addProperty(pname, priorities[i]->ydot_max);

        ssName.clear();
        ssName << "inequalities_" << i+1;
        ssName >> pname;
        this->ports()->addPort(pname, priorities[i]->inequalities_port).doc("inequalities indexes");
        this->properties()->addProperty(pname, priorities[i]->inequalities);

        priorities[i]->A.resize(nc, nq);
        priorities[i]->Wy = Eigen::MatrixXd::Identity(nc, nc);
        priorities[i]->ydot.resize(nc);
        priorities[i]->ydot_max.resize(0);
        priorities[i]->inequalities.resize(0);
    }
//    std::cout << "Itasc_solver_wdlspriorvel_test constructed !" <<std::endl;
}


bool Itasc_solver_wdlspriorvel_test::configureHook()
{
    Wq = Eigen::MatrixXd::Identity(nq, nq);

    // TODO Priority:configureHook
    for (unsigned int i=0;i<priorityNo;i++)
    {
        int nc = nc_priorities[i];
        priorities[i]->Wy = Eigen::MatrixXd::Identity(nc, nc);
        priorities[i]->A = priorities[i]->A_kdl.data;
    }

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

//  std::cout << "Itasc_solver_wdlspriorvel_test configured !" <<std::endl;
  return true;
}

bool Itasc_solver_wdlspriorvel_test::startHook(){
//  std::cout << "Itasc_solver_wdlspriorvel_test started !" <<std::endl;
  return true;
}

void Itasc_solver_wdlspriorvel_test::updateHook()
{
    if(testDone == true)
      return;

    //put dummy data on port
    Wq_port.write(Wq);

    // TODO Priority:updateHook
    for (unsigned int i=0;i<priorityNo;i++)
    {
        priorities[i]->A_port.write(priorities[i]->A);
        priorities[i]->Wy_port.write(priorities[i]->Wy);
        priorities[i]->ydot_port.write(priorities[i]->ydot);
        priorities[i]->ydot_max_port.write(priorities[i]->ydot_max);

    //assertion: we only use inequalities if the solver can handle it

    //TODO: the solver doesn't have inequalityProvisions, but there are inequalities present
    //TaskContext* solver_ptr = getPeer("Solver");
    //if((!solver_ptr->inequalityProvisions) && inequalities_1.size() != 0)
    //  log(Error) << "[[TestSolver] error: The problem has inequalities, but solver can't handle inequalities" << endlog();
    //else
        priorities[i]->inequalities_port.write(priorities[i]->inequalities);
    }

    //call solve()
    solve();

    //read from port
    if( qdot_port.read(qdot) != RTT::NoData)
    {
        // Validate the received solution.
        if( (compare(qdot, qdot_expected, 1e-9)) )
        {
          std::cout << " ** Test Success ! **" << std::endl;
          // Todo: interrupt properly
          //  ie not std::exit(0);
        }
        else
        {
          std::cerr.precision(12);
          std::cerr << " ** Test Failure ! **" << std::endl;
          std::cerr << " qdot " << qdot.transpose() << std::endl;
          std::cerr << " qdot_expected " << qdot_expected.transpose() << std::endl;
          std::cerr << " diff = " << (qdot_expected - qdot).transpose() << std::endl;
    //      std::exit(-1);
        }
    }
}

void Itasc_solver_wdlspriorvel_test::stopHook() {
  std::cout << "Itasc_solver_wdlspriorvel_test executes stopping !" <<std::endl;
  testDone = false;
}

void Itasc_solver_wdlspriorvel_test::cleanupHook() {
  std::cout << "Itasc_solver_wdlspriorvel_test cleaning up !" <<std::endl;
}

Itasc_solver_wdlspriorvel_test::Priority::Priority()
: A_kdl(Jacobian(7))
{
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
