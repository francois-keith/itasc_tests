/*******************************************************************************
 *                 This file is part of the OROCOS project                     *
 *                                                                             *
 *                          (C) 2011 Pieterjan Bartels                         *
 *                          (C) 2011 Dominick Vanthienen                       *
 *                          nick.vanthienen[at]mech.kuleuven.be,               *
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

#ifndef _ITASC_ROBOT_PR2_TEST_HPP_
#define _ITASC_ROBOT_PR2_TEST_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <Eigen/Core>

namespace iTaSC {

using namespace RTT;
using namespace KDL;
using namespace Eigen;

	class itasc_robot_pr2_test: public RTT::TaskContext {
        public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                itasc_robot_pr2_test(const std::string& name = "itasc_robot_pr2_test");
                ~itasc_robot_pr2_test(){};

		virtual bool checkPoses();
		virtual bool checkJointValues();
		
		virtual bool configureHook();
	
	private:
		RTT::InputPort<sensor_msgs::JointState> joint_state_port;
		RTT::InputPort<KDL::Frame> T_b_e_port;
		RTT::InputPort<std::vector<std::string> > q_names_in;
		RTT::InputPort<KDL::JntArray> q_port_in;	
		///input: vector of the names of object frames to broadcast
		RTT::InputPort<std::vector<std::string> > objectFrames_port;

		RTT::OutputPort<std::string> pose_check_to_test;
		RTT::OutputPort<std::string> jnt_value_check_to_test;

		//objectFrames to broadcast
		std::string externalName;
		std::vector<std::string> objectFramesToBC;	
		KDL::JntArray q_array;
		std::vector<std::string> q_names;
		sensor_msgs::JointState jntstate;
		RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&,const std::string&)> lookupTransform;
		geometry_msgs::TransformStamped stfm;
		std::string base_frame;	
		KDL::Frame ros_kdl_frame;
		KDL::Frame itasc_kdl_frame;
		double epsilon;
	};
}
#endif
