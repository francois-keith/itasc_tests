--           This file is part of the iTaSC project
--
--		    (C) 2012 Pieterjan Bartels
--                  (C) 2011 Dominick Vanthienen
--              dominick.vanthienen@mech.kuleuven.be,
--              Department of Mechanical Engineering,
--             Katholieke Universiteit Leuven, Belgium.
--                    http://www.orocos.org/itasc
--
-- You may redistribute this software and/or modify it under either the
-- terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1
-- <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your
-- discretion) of the Modified BSD License:
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
-- 1. Redistributions of source code must retain the above copyright
-- notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
-- 3. The name of the author may not be used to endorse or promote
-- products derived from this software without specific prior written
-- permission.
-- THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
-- IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
-- INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
-- OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
-- HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
-- STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
-- IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.

require "rttlib"
require "rfsm"
require "rfsm_emem" --needed for event memory
require "rfsm_rtt"
require "rfsm_ext" --needed for the sequential-AND state
require "rfsmpp"    --needed for the sequential-AND state
require "kdlpp"    --kdl pritty print (should be included in lua path!)

-- tc=Task Context of the compent we are in ( in this case itasc_robot_pr2_test)
tc=rtt.getTC()
local common_events_in, priority_events_in
local i

function configureHook()
	-- Peer table (to enable smaller code to request operations)
	-- Mappeers does an in dept search (also looks for peers of peers...)
	TestSupPeertable = rttlib.mappeers(function (tc) return tc end, tc)
	--print("[TestSuperVisor.lua] TestSuperVisor has following peers:")
	--for K,V in pairs(TestSupPeertable) do print( K) end
	
	-- PROPERTIES
	-- create a vector (table) of objectframe_names. ONLY INSERT STRINGS
	objectFrames_from_file=rtt.Property("strings", "OF_from_file", "vector with objectframe names")
	tc:addProperty(objectFrames_from_file)

	--array indexing for moveToNextPosition
	i = 0

	-- create a vector (table) of poses. ONLY INSERT KDL.FRAMES
	poses_from_file=rtt.Property("motion_control_msgs.JointPositions[]", "poses_from_file", "vector with poses")
	tc:addProperty(poses_from_file)
	
	-- create properties for epsilon_matrix and epsilon_scalar
	epsilon_matrix = rtt.Property("double", "epsilon_matrix", "maximum error for matrix comparison")
	tc:addProperty(epsilon_matrix)
	epsilon_scalar = rtt.Property("double", "epsilon_scalar", "maximum error for scalar comparison")
	tc:addProperty(epsilon_scalar)

	-- FSM
	-- load state machine
	fsm = rfsm.init(rfsm.load("scripts/test_fsm.lua"))	


	-- INPUT PORTS

	-- the following creates a string input port, adds it as a
	-- port to the Taskcontext. The third line generates a
	-- getevents function, which returns all data on the current port as
	-- events. This function is called by the rFSM core to check for
	-- new events.
	common_events_in = rtt.InputPort("string")
	tc:addPort(common_events_in, "test_common_events_in", "rFSM common_event input port")

	-- the following creates a string input port, adds it as an event
	-- driven port to the Taskcontext. The third line generates a
	-- getevents function, which returns all data on the current port as
	-- events. This function is called by the rFSM core to check for
	-- new events. (usefull for eg. e_stop event)
	priority_events_in = rtt.InputPort("string")
	tc:addEventPort(priority_events_in, "test_priority_events_in", "rFSM priority_event input port")

	-- TRIGGER events: the following creates a string input port, adds it as an event
	-- driven port to the Taskcontext.
	trigger_events_in = rtt.InputPort("string")
	tc:addEventPort(trigger_events_in, "test_trigger_events_in", "test trigger_event input port")

	-- get all events from the all input ports
	fsm.getevents = rfsm_rtt.gen_read_str_events(common_events_in, priority_events_in, trigger_events_in)

	-- OUTPUT PORTS

	-- create a string port with which the objectframes are sent. table of strings
	objectframes_out_port = rtt.OutputPort("strings")
	tc:addPort(objectframes_out_port, "objectframes_out", "objectframes outport")

	-- create a string port with which the current common events are send
	common_events_out = rtt.OutputPort("string")
	tc:addPort(common_events_out, "test_common_events_out", "current common events in TestFSM")

	-- create an event driven string port with which the current priority events are send
	priority_events_out = rtt.OutputPort("string")
	tc:addPort(priority_events_out, "test_priority_events_out", "current priority events in TestFSM")

	-- create a string port with which the current TRIGGER events are send
	trigger_events_out = rtt.OutputPort("string")
	tc:addPort(trigger_events_out, "test_trigger_events_out", "current trigger_events in TestFSM")

	-- create a port with which the desired joint positions are sent
	joint_positions_out = rtt.OutputPort("motion_control_msgs.JointPositions")
	tc:addPort(joint_positions_out, "joint_positions_out", "desired positions to nAxesGenerator")

	-- optional: create a string port to which the currently active
	-- state of the FSM will be written. gen_write_fqn generates a
	-- function suitable to be added to the rFSM step hook to do this
	fqn_out = rtt.OutputPort("string")
	tc:addPort(fqn_out, "currentState", "current active rFSM state")
	fsm.step_hook=rfsm_rtt.gen_write_fqn(fqn_out)

	--raise event functions
	raise_common_event=rfsm_rtt.gen_raise_event(common_events_out, fsm)
	raise_priority_event=rfsm_rtt.gen_raise_event(priority_events_out, fsm)
	raise_trigger_event=rfsm_rtt.gen_raise_event(trigger_events_out, fsm)

	

	return true
end


function updateHook()
	rfsm.run(fsm)
end

function cleanupHook()
	-- cleanup the created ports.
	tc:removePort(common_events_in:info().name)
	-- cleanup created variables
	common_events_in:delete()
	priority_events_in:delete()
end

-- CONFIGURE
--- Function containing RTT specific info to configure pr2Robot
function configurePr2Robot()
	if TestSupPeertable.Pr2Robot:configure() 
        then --print("   pr2Robot configured") 
        else print("    [" .. tc:getName() .. "]:function configurePr2Robot(): couldn't configure Pr2Robot")
	     raise_common_event("e_emergency") end
end




--- Function containing RTT specific info to configure pr2connect
function configurePr2Connect()
	if TestSupPeertable.pr2connector:configure() 
        then return true
        else --print("    [" .. tc:getName() .. "]:function configurePr2Connect(): couldn't configure pr2connector")
	     return false
	end
end


--- Function containing RTT specific info to configure TestComponent
function configureTestComponent()
	if TestSupPeertable.TestComponent:configure() 
        then --print("   cartesian_generator configured") 
        else print("    [" .. tc:getName() .. "]:function configureTestComponent(): couldn't configure TestComponent")
	     raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to configure TrajectoryGenerator
function configureTrajectoryGenerator()
	if TestSupPeertable.nAxes_generator:configure() 
        then --print("   nAxes_generator configured") 
        else print("    [" .. tc:getName() .. "]:function configureTrajectoryGenerator(): couldn't configure nAxes_generator")
	     raise_common_event("e_emergency") end
end


function configurereporter()
	if TestSupPeertable.reporter:configure() 
        then --print("   nAxes_generator configured") 
        else print("    [" .. tc:getName() .. "]:function configureTrajectoryGenerator(): couldn't configure nAxes_generator")
	     raise_common_event("e_emergency") end
end

function startreporter()
	if TestSupPeertable.reporter:start() 
        then --print("   cartesian_generator started") 
        else print("    [" .. tc:getName() .. "]:function startPr2Robot(): couldn't start Pr2Robot")
	     raise_common_event("e_emergency") 
	end
end

--START
--- Function containing RTT specific info to start Pr2Robot
function startPr2Robot()
	if TestSupPeertable.Pr2Robot:start() 
        then --print("   cartesian_generator started") 
        else print("    [" .. tc:getName() .. "]:function startPr2Robot(): couldn't start Pr2Robot")
	     raise_common_event("e_emergency") 
	end
end

--- Function containing RTT specific info to start Pr2Connect
function startPr2Connect()
	if TestSupPeertable.pr2connector:start() 
        then --print("   cartesian_generator started") 
        else print("    [" .. tc:getName() .. "]:function startPr2Connect(): couldn't start Pr2Connect")
	     raise_common_event("e_emergency") 
	end
end
--- Function containing RTT specific info to start TestComponent
function startTestComponent()
	if TestSupPeertable.TestComponent:start() 
        then --print("   cartesian_generator started") 
        else print("    [" .. tc:getName() .. "]:function startTestComponent(): couldn't start TestComponent")
	     raise_common_event("e_emergency") 
	end
end
--- Function containing RTT specific info to start TrajectoryGenerators
function startTrajectoryGenerator()
	if TestSupPeertable.nAxes_generator:start() 
        then --print("   nAxes_generator started") 
        else print("    [" .. tc:getName() .. "]:function startTrajectoryGenerator(): couldn't start nAxes_generator")
	     raise_common_event("e_emergency") 
	end
end


--STOP
--- Function containing RTT specific info to stop Pr2Robot
function stopPr2Robot()
	if TestSupPeertable.Pr2Robot:stop() 
        then --print("   cartesian_generator stopped") 
        else print("    [" .. tc:getName() .. "]:function stopPr2Robot(): couldn't stop Pr2Robot")
	     raise_common_event("e_emergency") 
	end
end

--- Function containing RTT specific info to stop Pr2Connect
function stopPr2connector()
	if TestSupPeertable.pr2connector:stop() 
        then --print("   cartesian_generator stopped") 
        else print("    [" .. tc:getName() .. "]:function stopPr2Connect(): couldn't stop Pr2Connect")
	     raise_common_event("e_emergency") 
	end
end

--- Function containing RTT specific info to stop TestComponent
function stopTestComponent()
	if TestSupPeertable.TestComponent:stop() 
        then --print("   cartesian_generator stopped") 
        else print("    [" .. tc:getName() .. "]:function stopTestComponent(): couldn't stop TestComponent")
	     raise_common_event("e_emergency") 
	end
end

--- Function containing RTT specific info to stop TrajectoryGenerator
function stopTrajectoryGenerator()
	if TestSupPeertable.nAxes_generator:stop() 
        then --print("   nAxes_generator stopped") 
        else print("    [" .. tc:getName() .. "]:function stopTrajectoryGenerator(): couldn't stop nAxes_generator")
	     raise_common_event("e_emergency") 
	end
end

--- Function to kill the application
function killTest(status)
    os.exit(status)
end

--------------------

--- Function containing RTT specific info to move pr2robot to some pose.
function moveToNextPosition()
	print("move to next entered")
	temp = poses_from_file:get()
	print("temp read in")
	if(i < temp.size )
	then print(temp[i])
	     joint_positions_out:write(temp[i])
	     i = i + 1
	else raise_common_event("e_test_done") --all moves are done
	end
end

--- Function containng RTT specific info to move pr2robot to some pose.
function doPoseChecks()
	if TestSupPeertable.TestComponent:checkPoses(epsilon_matrix:get())
	then --do nothing 
	else print ("    pose checks did not match!")
	     raise_common_event("e_checkError")
	end
end

--- Function containing RTT specific info to move pr2robot to some pose.
function doJointValueChecks()
	if TestSupPeertable.TestComponent:checkJointValues(epsilon_scalar:get())
	then --do nothing 
	else print ("    jointvalue checks did not match!")
	     raise_common_event("e_checkError")
	end
end

function passPositionToGen()
	TestSupPeertable.TestComponent:passPositionToGen()
end
function updateRobot()
	TestSupPeertable.Pr2Robot:updateRobotState()
end

function sendToRobot()
	TestSupPeertable.Pr2Robot:sendToRobot()
end

-- connect the robot's ports
function connect_ports()
	--runScript=tc:provides("scripting"):getOperation("runScript")
	--runScript("../../../iTaSC_pr2/scripts/connectToControllers.ops")
	-- doesn't work, "No method "stream" registered for the object or task 'ApplicationSuperVisor'"
--print(tc:provides())
--loadPrograms=tc:provides("scripting"):getOperation("loadPrograms")
--loadPrograms("../../../iTaSC_pr2/scripts/connectToControllers.ops")

	-- lua:
	-- all assuming the iTaSC::pr2connect component is named 'pr2connector'
	local string pr2connector = "pr2connector"
    rtt.logl('Info',"[" .. tc:getName() .. "] Connecting PR2 to " .. pr2connector)
	depl = tc:getPeer("Deployer")
	-- ROS connection policy
	roscp=rtt.Variable("ConnPolicy")
	roscp.transport = 3
	
	-- INPUT
	-- connect the jointstate topic to the pr2connector
	roscp.name_id = "/joint_states"
	depl:stream(pr2connector .. ".joint_state_from_robot", roscp)
	

	-- OUTPUT
	roscp.name_id = "/l_shoulder_pan_velocity_controller/command"
	depl:stream(pr2connector .. ".l_shoulder_pan_joint_qdot", roscp)
	roscp.name_id = "/l_shoulder_lift_velocity_controller/command"
	depl:stream(pr2connector .. ".l_shoulder_lift_joint_qdot", roscp)
	roscp.name_id = "/l_upper_arm_roll_velocity_controller/command"
	depl:stream(pr2connector .. ".l_upper_arm_roll_joint_qdot", roscp)
	roscp.name_id = "/l_elbow_flex_velocity_controller/command"
	depl:stream(pr2connector .. ".l_elbow_flex_joint_qdot", roscp)
	roscp.name_id = "/l_forearm_roll_velocity_controller/command"
	depl:stream(pr2connector .. ".l_forearm_roll_joint_qdot", roscp)
	roscp.name_id = "/l_wrist_flex_velocity_controller/command"
	depl:stream(pr2connector .. ".l_wrist_flex_joint_qdot", roscp)
	roscp.name_id = "/l_wrist_roll_velocity_controller/command"
	depl:stream(pr2connector .. ".l_wrist_roll_joint_qdot", roscp)

	roscp.name_id = "/r_shoulder_pan_velocity_controller/command"
	depl:stream(pr2connector .. ".r_shoulder_pan_joint_qdot", roscp)
	roscp.name_id = "/r_shoulder_lift_velocity_controller/command"
	depl:stream(pr2connector .. ".r_shoulder_lift_joint_qdot", roscp)
	roscp.name_id = "/r_upper_arm_roll_velocity_controller/command"
	depl:stream(pr2connector .. ".r_upper_arm_roll_joint_qdot", roscp)
	roscp.name_id = "/r_elbow_flex_velocity_controller/command"
	depl:stream(pr2connector .. ".r_elbow_flex_joint_qdot", roscp)
	roscp.name_id = "/r_forearm_roll_velocity_controller/command"
	depl:stream(pr2connector .. ".r_forearm_roll_joint_qdot", roscp)
	roscp.name_id = "/r_wrist_flex_velocity_controller/command"
	depl:stream(pr2connector .. ".r_wrist_flex_joint_qdot", roscp)
	roscp.name_id = "/r_wrist_roll_velocity_controller/command"
	depl:stream(pr2connector .. ".r_wrist_roll_joint_qdot", roscp)

	roscp.name_id = "/base_controller/command"
	depl:stream(pr2connector .. ".base_combinedtwist", roscp)
	roscp.name_id = "/torso_lift_velocity_controller/command"
	depl:stream(pr2connector .. ".torso_lift_joint_qdot", roscp)

    roscp.name_id = "/head_pan_velocity_controller/command"
    depl:stream(pr2connector .. ".head_pan_joint_qdot", roscp)
    roscp.name_id = "/head_tilt_velocity_controller/command"
    depl:stream(pr2connector .. ".head_tilt_joint_qdot", roscp)

	-- delta qdot inputs
	roscp.name_id = "/l_shoulder_pan_velocity_controller/state"
	depl:stream(pr2connector .. ".l_shoulder_pan_joint_e", roscp)
	roscp.name_id = "/l_shoulder_lift_velocity_controller/state"
	depl:stream(pr2connector .. ".l_shoulder_lift_joint_e", roscp)
	roscp.name_id = "/l_upper_arm_roll_velocity_controller/state"
	depl:stream(pr2connector .. ".l_upper_arm_roll_joint_e", roscp)
	roscp.name_id = "/l_elbow_flex_velocity_controller/state"
	depl:stream(pr2connector .. ".l_elbow_flex_joint_e", roscp)
	roscp.name_id = "/l_forearm_roll_velocity_controller/state"
	depl:stream(pr2connector .. ".l_forearm_roll_joint_e", roscp)
	roscp.name_id = "/l_wrist_flex_velocity_controller/state"
	depl:stream(pr2connector .. ".l_wrist_flex_joint_e", roscp)
	roscp.name_id = "/l_wrist_roll_velocity_controller/state"
	depl:stream(pr2connector .. ".l_wrist_roll_joint_e", roscp)

	roscp.name_id = "/r_shoulder_pan_velocity_controller/state"
	depl:stream(pr2connector .. ".r_shoulder_pan_joint_e", roscp)
	roscp.name_id = "/r_shoulder_lift_velocity_controller/state"
	depl:stream(pr2connector .. ".r_shoulder_lift_joint_e", roscp)
	roscp.name_id = "/r_upper_arm_roll_velocity_controller/state"
	depl:stream(pr2connector .. ".r_upper_arm_roll_joint_e", roscp)
	roscp.name_id = "/r_elbow_flex_velocity_controller/state"
	depl:stream(pr2connector .. ".r_elbow_flex_joint_e", roscp)
	roscp.name_id = "/r_forearm_roll_velocity_controller/state"
	depl:stream(pr2connector .. ".r_forearm_roll_joint_e", roscp)
	roscp.name_id = "/r_wrist_flex_velocity_controller/state"
	depl:stream(pr2connector .. ".r_wrist_flex_joint_e", roscp)
	roscp.name_id = "/r_wrist_roll_velocity_controller/state"
	depl:stream(pr2connector .. ".r_wrist_roll_joint_e", roscp)

	roscp.name_id = "/base_controller/state"
	depl:stream(pr2connector .. ".base_combinedtwist", roscp)
	roscp.name_id = "/torso_lift_velocity_controller/state"
	depl:stream(pr2connector .. ".torso_lift_joint_e", roscp)

    roscp.name_id = "/head_pan_velocity_controller/state"
    depl:stream(pr2connector .. ".head_pan_joint_e", roscp)
    roscp.name_id = "/head_tilt_velocity_controller/state"
    depl:stream(pr2connector .. ".head_tilt_joint_e", roscp)
	
end

function configureObjectframes()
	TestSupPeertable.Pr2Robot:configureObjectFrames()
end

--- Function containing RTT specific info to unlock robots
function unlockRobots()
    if TestSupPeertable.pr2connector:unlockAllAxes() 
    then print("   [" .. tc:getName() .. " axes unlocked!") 
    else 
        print("   [" .. tc:getName() .. " unable to unlock axes!")
        raise_common_event("e_emergency")  
    end 
end

--- Function containing RTT specific info to unlock robots
function lockRobots()
    if TestSupPeertable.pr2connector:lockAllAxes() then print("   [" .. tc:getName() .. " axes locked!") else raise_common_event("e_emergency")  end
end
