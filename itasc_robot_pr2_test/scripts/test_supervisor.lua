--           This file is part of the iTaSC project                      
--                                                                       
--                  (C) 2012 Dominick Vanthienen                         
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
require "rfsm_ext"  --needed for the sequential-AND state
require "rfsmpp"    --needed for the sequential-AND state 
require "kdlpp"     --kdl pritty print (should be included in lua path!)
require "rttros"    --needed for 'find_rospack'

tc=rtt.getTC()
local timer_id_in_fs ,timer_id_in

local prev_timertrigger_time={}
local current_timertrigger_time={}

function configureHook() 
	
	-- peer table (to enable smaller code to request operations)
	peertable = rttlib.mappeers(function (tc) return tc end, tc)
	--print("[itascSuperVisor.lua] SuperVisor has following peers:")
	--for K,V in pairs(peertable) do print( K) end
	
	-- PROPERTIES
	-- create a timer id property
	test_timer_id=rtt.Property("int", "test_timer_id", "Timer ID of the application timer")
	tc:addProperty(test_timer_id)
	-- location of the test_fsm
    test_fsm_package_prop=rtt.Property("string","test_package","package where to find the test_fsm, if any")
    tc:addProperty(test_fsm_package_prop)
	test_fsm_prop=rtt.Property("string", "test_fsm", "path and name to the test_fsm file, starting from the package (start with a slash), if any")
	tc:addProperty(test_fsm_prop)
    -- object frames to test
    objectFrames=rtt.Property("strings","objectFrames","object frames to test")
    tc:addProperty(objectFrames)

    -- fill in standard values for the properties
    application_timer_id:set(1)
    test_fsm_package_prop:set("")
	test_fsm_prop:set(rttros.find_rospack("itasc_robot_pr2_test") .. "/scripts/test_fsm.lua")
	
	-- INPUT PORTS 
	-- Port to recieve trigger from a timer
	time_trigger = rtt.InputPort("int")
	tc:addEventPort(time_trigger,"trigger","Port to recieve trigger from a timer")
    objectFramesPort = rttlib.port_clone_conn(peertable.pr2robot:getPort("objectFramesPort"))

	--OUTPUT PORTS
	-- create a string port to which the currently active
	-- state of the FSM will be written. 
	fqn_out = rtt.OutputPort("string")
	tc:addPort(fqn_out, "currentState", "current active rFSM state")

	--raise event functions
	raise_common_event=gen_raise_event(common_events_out, fsm)
	raise_priority_event=gen_raise_event(priority_events_out, fsm)	
	raise_trigger_event=gen_raise_event(trigger_events_out, fsm)

	return true	
end

function startHook()
    print("[test_supervisor.lua] starting") 
    -- getting the file locations 
    if(test_fsm_package_prop:get()=="")
    then
        test_fsm_file = test_fsm_prop:get()
    else
        test_fsm_file = rttros.find_rospack(test_fsm_package_prop:get()) .. test_fsm_prop:get()
    end
    
    -- get the object frames to test
    if(objectFrames:get()=="")
    then
        print("   ERROR: unable to find objectFrames to test")
        return false
    end

	-- load state machine
	print("[test_supervisor.lua] loading FSM: " .. test_fsm_file )
	fsm = rfsm.init(rfsm.load(test_fsm_file))

	-- String port to which the currently active
	-- state of the FSM will be written. gen_write_fqn generates a
	-- function suitable to be added to the rFSM step hook to do this.
	rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(fqn_out))
	
    return true
end

function updateHook() 
	timer_id_in_fs, timer_id_in = time_trigger:read()
	
	if timer_id_in_fs=="NewData" then
		if timer_id_in==application_timer_id:get()then
			rfsm.send_events(fsm, 'e_TimerTrigger')
			rfsm.run(fsm)
		end
	else 
		rfsm.run(fsm)
	end 
end

function cleanupHook()
	-- cleanup the created ports.
	tc:removePort(fqn_out:info().name)     
	-- cleanup created properties
	tc:removeProperty("test_timer_id")   
end

-- connect the robot's ports
function connect_ports()
	-- lua:
	-- all assuming the iTaSC::pr2connect component is named 'pr2connector'
	local string pr2connector = "pr2connector"
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

--- Function containing RTT specific info to configure RTTtf
function configureRTTtf()		
	if peertable.rtt_tf:configure() then print("   rtt_tf configured") 
    else 
		print("   ERROR: Unable to configure RTTtf") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to  configure pr2robot
function configurePR2robot()
	if peertable.pr2robot:configure() then print("   PR2robot configured") 
 	else 
		print("   ERROR: Unable to configure PR2robot") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to  configure pr2connector
function configurePR2connector()
	if peertable.pr2connector:configure() then print("   PR2connector configured") 
 	else 
		print("   ERROR: Unable to configure PR2connector") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to 
function broadcastObjectFrames()		
	objectFramesPort:write(objectFrames:get()) 
end

--- Function containing RTT specific info to 
function configureObjectFrames()		
    if peertable.pr2robot:configureObjectFrames()
    then 
        print("   object frames configured")
    else
        print("   ERROR: Unable to configure object frames")
        raise_common_event("e_emergency")
    end
end

--- Function containing RTT specific info to start RTTtf
function startRTTtf()		
	if peertable.rtt_tf:start() then print("   rtt_tf started") 
 	else 
		print("   ERROR: Unable to start rtt_tf") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to  start pr2robot
function startPR2robot()
	if peertable.pr2robot:start() then print("   PR2robot started") 
 	else 
		print("   ERROR: Unable to start PR2robot") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to  start pr2connector
function startPR2connector()
	if peertable.pr2connector:start() then print("   PR2connector started") 
 	else 
		print("   ERROR: Unable to start pr2connector") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to unlock robots
function unlockRobots()
	if peertable.pr2connector:unlockAllAxes() 
	then print("   PR2connector: axes unlocked!") 
	else 
		print("   ERROR: PR2connector: unable to unlock axes")
		raise_common_event("e_emergency")
	end
end

-- RUNNING

-- 
function updateRobotState()
    peertable.pr2robot:updateRobotState() 
end

--
function sendToRobot()
    peertable.pr2robot:sendToRobot() 
end

-- STOPPING

--- Function containing RTT specific info to start RTTtf
function stopRTTtf()		
	if peertable.rtt_tf:stop() then print("   rtt_tf stopped") 
 	else 
		print("   ERROR: Unable to stop rtt_tf") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to  stop pr2robot
function stopPR2robot()
	if peertable.pr2robot:stop() then print("   PR2robot stopped") 
 	else 
		print("   ERROR: Unable to stop pr2robot") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to  stop pr2connector
function stopPR2connector()
	if peertable.pr2connector:stop() then print("   PR2connector stopped") 
 	else 
		print("   ERROR: Unable to stop pr2connector") 
 	 	raise_common_event("e_emergency")
 	end
end

--- Function containing RTT specific info to unlock robots
function lockRobots()
	if peertable.pr2connector:lockAllAxes() then print("   PR2connector: axes locked!") 
    else 
		print("   ERROR: Unable to lock robot axes")  
        raise_common_event("e_emergency")
    end
end
