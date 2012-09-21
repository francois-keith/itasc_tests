/*******************************************************************************
 *                 This file is part of the OROCOS project                     *
 *                          						       *
 *                          (C) 2012 Pieterjan Bartels                         *
 *                          (C) 2011 Dominick Vanthienen                       *
 *                          nick.vanthienen[at]mech.kuleuven.be                *
 *                    Department of Mechanical Engineering,                    *
 *                   Katholieke Universiteit Leuven, Belgium.                  *
 *                         http://www.orocos.org/itasc                         *
 *                                                                             *
 *       You may redistribute this software and/or modify it under either the  *
 *       terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
 *       <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
 *       discretion) of the Modified BSD License:                              *
 *       Redistribution and use in source and binary forms, with or without    *
 *       modification, are permitted provided that the following conditions    *
 *       are met:                                                              *
 *       1. Redistributions of source code must retain the above copyright     *
 *       notice, this list of conditions and the following disclaimer.         *
 *       2. Redistributions in binary form must reproduce the above copyright  *
 *       notice, this list of conditions and the following disclaimer in the   *
 *       documentation and/or other materials provided with the distribution.  *
 *       3. The name of the author may not be used to endorse or promote       *
 *       products derived from this software without specific prior written    *
 *       permission.                                                           *
 *       THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
 *       IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
 *       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
 *       ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
 *       INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
 *       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
 *       OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
 *       HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
 *       STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
 *       IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
 *       POSSIBILITY OF SUCH DAMAGE.                                           *
 *                                                                             *
 *******************************************************************************/
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include "itasc_robot_pr2_test.hpp"
#include <tf/exceptions.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>


ORO_CREATE_COMPONENT(iTaSC::itasc_robot_pr2_test);

namespace iTaSC {
itasc_robot_pr2_test::itasc_robot_pr2_test(const std::string& name)
	:TaskContext(name),
	epsilon(10^-12),
	base_frame("odom_combined") {
	//ports
	//input
	this->ports()->addPort("q_port_in", q_port_in);
	this->ports()->addPort("q_names_in", q_names_in).doc(	
			"read in names of joints from itasc_pr2");
	this->ports()->addPort("joint_state_from_robot", joint_state_port).doc(
		"name, position, velocity and effort of all the joints of the robot");
	this->ports()->addPort("objectFramesPort", objectFrames_port);
	this->ports()->addEventPort("qdot_in", desired_pos_in, boost::bind(&itasc_robot_pr2_test::convertJointPositions, this) );

	//output
	this->ports()->addPort("pose_to_test", pose_check_to_test);
	this->ports()->addPort("jnt_to_test", jnt_value_check_to_test);
	this->ports()->addPort("qdot_out", qdot_out);
	this->ports()->addPort("msr_pos_out", msr_pos_out);
	this->properties()->addProperty("epsilon", epsilon);
	this->properties()->addProperty("base_frame",base_frame).doc(
		"the name of the base frame of the pr2");

	this->addOperation("passPositionToGen", &itasc_robot_pr2_test::passPositionToGen, this, ClientThread);
	this->addOperation("checkPoses", &itasc_robot_pr2_test::checkPoses, this, ClientThread);
	this->addOperation("checkJointValues", &itasc_robot_pr2_test::checkJointValues, this, ClientThread);

}

bool itasc_robot_pr2_test::configureHook(){
	Logger::In in(this->getName());

	// provide rtt_tf operation, needed to get transform from base to objectframe...
	if(!this->hasPeer("rtt_tf")){
		log(Error) << "component has no peer rtt_tf " << endlog();
		return false;
	}
	if(!(this->getPeer("rtt_tf")->operations()->hasMember("lookupTransform")) ){
		log(Error) << "component peer rtt_tf has no operation lookupTransform " << endlog();
		return false;
	}
	lookupTransform = this->getPeer("rtt_tf")->provides()->getOperation("lookupTransform");

	ros_kdl_frame = KDL::Frame::Identity();
	temp_qdot_out.resize(20);
	temp_desired_pos.velocities.resize(20);

	//read vector with object_frame names
	if(NoData==objectFrames_port.read(objectFramesToBC)){
		log(Error) << "No data on the objectFramesPort!" << endlog();
		return false;
	}

	T_b_e_ports.reserve(objectFramesToBC.size());
	
	// for all objectframes
	for(unsigned int j = 0; j < objectFramesToBC.size(); j++){
		//compose name of port at robot
		externalName = "Pose_" + objectFramesToBC[j] + "_base";
		T_b_e_ports.push_back(new InputPort<KDL::Frame>());
		this->ports()->addPort(objectFramesToBC[j] + "_port", * T_b_e_ports[j]);
		//connect local port to external port
		T_b_e_ports[j]->connectTo(this->getPeer("Pr2Robot")->ports()->getPort(externalName));
	}

	
	return true;
}

bool itasc_robot_pr2_test::checkPoses() {
	

	// for all objectframes
	for(unsigned int j = 0; j < objectFramesToBC.size(); j++){
		//read in Frame from itasc_pr2 
		T_b_e_ports[j]->read(itasc_kdl_frame);
		//get KDL::Frame from ros topic
		try
		{
			//do transform from base to objectframe
			stfm = lookupTransform(base_frame,objectFramesToBC[j]);
		}catch (tf::TransformException ex)
		{
			log(Error) << "failed at number: " << j << " catched an error in rtt_tf::lookupTransform: between: "  << endlog();
			log(Error) << "base_frame: " << base_frame << "objectframe: " << objectFramesToBC[j] << "\n" << ex.what() << endlog(); 
		}
		ros_kdl_frame.p = KDL::Vector(stfm.transform.translation.x,stfm.transform.translation.y,stfm.transform.translation.z);
		ros_kdl_frame.M = KDL::Rotation::Quaternion(stfm.transform.rotation.x,stfm.transform.rotation.y,stfm.transform.rotation.z,stfm.transform.rotation.w);
		
		if(!KDL::Equal(itasc_kdl_frame, ros_kdl_frame, epsilon)) {
				log(Warning) << "itasc frame \n" << itasc_kdl_frame << "ros frame \n" << ros_kdl_frame << endlog();
				log(Warning) << "e_check of poses failed, frames weren't equal for objectframe :" << objectFramesToBC[j] << endlog();
				return false;

		}

	}
	log(Warning) << "checkPoses: before write" << endlog();
	//send event
	pose_check_to_test.write("e_check of poses completed, all KDL::Frames were equal");
	log(Warning) << "checkPoses: end" << endlog();
	return true;
}

bool itasc_robot_pr2_test::checkJointValues() {
	//get information from itasc_pr2	
	this->q_port_in.read(q_array);
	this->q_names_in.read(q_names);
	//get information from ROS_topic
	this->joint_state_port.read(jntstate);

	//turn jntstate into vector with names and vector with values: not necessary: just concatenate! 

	bool localresult;
	//compare the results: a combination of name and position from the itasc_pr2 should be the same as some combination from the ros_topic
	for(unsigned int i = 0; i < q_names.size();i++){
		localresult = false;
		for(unsigned int j =0; j < jntstate.name.size() && !localresult; j++) {
			if(q_names[i].compare(jntstate.name[j]) == 0){ //their names are equal! now the values should be too.
				localresult = true;
				if(q_array(i) != jntstate.position[j]){
					log(Error) << "values for joint " << q_names[i] << " didn't match!" << endlog();
					//send event by writing a string on output port	
					jnt_value_check_to_test.write("e_joint value check failed, joint values for joint " + q_names[i] + "weren't equal");
					return false; //values aren't equal!		
				}
			}	
		}
		if(!localresult){
			log(Error) << "couldn't find joint with name " << q_names[i] << " in vector with names from ROS topic" << endlog();
			//send event by‌ writing a string on output port
			jnt_value_check_to_test.write("e_joint value check failed, couldn't find a joint with name: " + q_names[i]);
			return false;
		}
		else {
		log(Info) << "values for joint " << q_names[i] << " matched." << endlog();		
		}
	}	
	//send event
	jnt_value_check_to_test.write("e_joint value check completed, all joint values were equal");
	return true;
}

void itasc_robot_pr2_test::convertJointPositions() {
	//read from port
	desired_pos_in.read(temp_desired_pos);
	//omzetten van temp_desired_pos (motion_control_msgs::jointpositions) to temp_qdot_out (KDL::JntArray)
	for(unsigned int j = 0; j < temp_desired_pos.velocities.size(); j++){
		temp_qdot_out(j) = temp_desired_pos.velocities[j];
	}

	//write solution
	qdot_out.write(temp_qdot_out);
}

void itasc_robot_pr2_test::passPositionToGen(){
	if(q_port_in.read(temp_qdot_in) == NoData){ //read in values
		log(Error) << "can't pass information to generator, no data on q_port_in" << endlog();
	}

	tempjointstate.position.resize(temp_qdot_in.rows());
	//omzetten van temp_desired_pos (motion_control_msgs::jointpositions) to temp_qdot_out (KDL::JntArray)
	for(unsigned int j = 0; j < temp_qdot_in.rows(); j++){
		 tempjointstate.position[j] = temp_qdot_in(j);
	}

	msr_pos_out.write(tempjointstate);
}

}
