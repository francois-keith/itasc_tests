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

function guardMultipleEvents(tr, taskTable, prefix, appendix)
	local x=false
	for i=1,#taskTable do
		local x_temp_name = prefix .. taskTable[i] .. appendix
		local x_temp = tr.src.emem[x_temp_name]
		if i>1 then
			x = x and x_temp and x_temp > 0
		else
			x = x_temp and x_temp > 0
		end
	end
	if x then return true end
	return false
end
		
return rfsm.state{
	
NONemergency = rfsm.state{

	ConfiguringTest = rfsm.state{	
		entry=function(fsm)
			print("=>testFSM=>ConfiguringTest state entry")
			configureRTTtf()
            configurePR2robot()
			-- configure pr2connector, pr2Robot MUST be configured before pr2robot, therefore halt configuring pr2connect
			configurePR2connector()
            broadcastObjectFrames()
            configureObjectFrames()
		end,
	},

	StartingTest = rfsm.state{
		entry=function(fsm)
			print("=>testFSM->StartingTest state entry")
			startRTTtf()
			startPR2connector()
			-- connect the ports of pr2connector to ros topics (the joint velocity controller commands)
			connect_ports()
			print("   PR2connector connected to PR2 hardware")
			unlockRobots()
			startPR2robot()
		end,
	},

	RunningTest = rfsm.state{
        updateStep = rfsm.state{
            entry = function()
                updateRobotState()
            end,
        },

        calculateStep = rfsm.state{
            
        },
    
        sendStep = rfsm.state{
            entry = function()
            sendToRobot()
            end,
        },

	    rfsm.transition { src='initial', tgt='updateStep' },
	    rfsm.transition { src='updateStep', tgt='calculateStep', events={'e_done'} },
	    rfsm.transition { src='calculateStep', tgt='sendStep', events={'e_done'} },
	    rfsm.transition { src='sendStep', tgt='updateStep', events={'e_done'} },
	},

	StoppingTest = rfsm.state{
		entry = function ()
			print("=>testFSM->StoppingTest state entry")
			stopPR2connector()
			lockRobots()	
			stopPR2robot()
			raise_trigger_event("e_stopTasks")
		end,
	},

	rfsm.transition { src='initial', tgt='ConfiguringTest' },
	rfsm.transition { src='ConfiguringTest', tgt='StoppingTest', events={'e_stopTest'} },
	rfsm.transition { src='ConfiguringTest', tgt='StartingTest', events={'e_done'} },
	rfsm.transition { src='StartingTest', tgt='StoppingTest', events={ 'e_stopTest' } },
	rfsm.transition { src='StartingTest', tgt='RunningTest', events={'e_done'} },
	rfsm.transition { src='RunningTest', tgt='StoppingTest', events={ 'e_stopTest' } },
},

TestEmergency = rfsm.state{
	entry = function ()
		stopAllComponents()
		raise_priority_event("e_emergency")
		print("[EMERGENCY] =>iTaSC application in emergency state!")
		print("[EMERGENCY] =>iTaSC has therefore raised an priority emergency event")
	end,
},

rfsm.transition { src='initial', tgt='NONemergency' },
rfsm.transition { src='NONemergency', tgt='TestEmergency', events={'e_emergency'} },
}
