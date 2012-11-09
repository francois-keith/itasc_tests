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

-- statemachine
require "rfsm_timeevent"
rfsm_timeevent.set_gettime_hook(rtt.getTime)
--require("ansicolors")
local state, trans, conn = rfsm.state, rfsm.trans, rfsm.conn

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

return rfsm.composite_state{
	--dbg = fsmpp.gen_dbgcolor3({["STATE_ENTER"]=true, ["STATE_EXIT"]=true, ["HIBERNATING"]=false, ["EXEC_PATH"]=false, ["EFFECT"]=false, ["DOO"]=false, ["CHECKING"]=false, ["RAISED"]=true},  false, ansicolors.yellow .. ansicolors.bright .. "ITASCfsm" ..  ansicolors.reset),

   dbg=rfsmpp.gen_dbgcolor("test_fsm", { STATE_ENTER=true, STATE_EXIT=true }, false),

NONemergency = rfsm.composite_state{
	PreOperational = rfsm.simple_state{
		entry=function()
			raise_common_event("e_configTest")
		end
	},

	ConfiguringFirstPartTest = rfsm.simple_state{
		entry=function()
			objectframes_out_port:write(objectFrames_from_file:get())
			configurePr2Robot()
			raise_common_event("e_FirstPartConfigured")
		end,
	},
	
	ConfiguringPr2Connect = rfsm.simple_state{
			doo = function()
				while true do
					if(configurePr2Connect())
					then raise_common_event("e_Pr2Configured")
					     rfsm.yield()
					else rfsm.yield()
					end
				end
			end,
	},
	
	errorOnConfigure = rfsm.simple_state{
		entry = function()
			raise_common_event('e_emergency')		
		end
	},
	
	ConfiguringSecondPart = rfsm.simple_state{
		entry = function()
			connect_ports()
            unlockRobots()
			configureTrajectoryGenerator()
			configureObjectframes()
			configureTestComponent()
			configurereporter()	
			raise_common_event("e_TestConfigured")
		end,
	},

	ConfiguredTest = rfsm.simple_state{
		entry=function()
			--print("=>iTaSCFSM->ConfiguredITASC state entry")
			raise_common_event("e_startUpTest")
		end,
	},

	StartingTest = rfsm.simple_state{
		entry=function(fsm)
			--print("=>iTaSCFSM->StartingITASC state entry")	
			startPr2Robot()
			startPr2Connect()
			--startTrajectoryGenerator() somewhere else!
			startTestComponent()	
			startreporter()
			raise_common_event("e_startTest")
		end,
	},

	StartedTest = rfsm.simple_state{
		entry=function()
			--print("=>iTaSCFSM->StartedITASC state entry")
			raise_common_event("e_runTests")
		end,
	},


	-- sequential AND state
	RunningTest = state {
	   Initializing =rfsm.simple_state{
	      entry=function(fsm)
		       --print("=>iTaSCFSM=>Running=>CompositeTaskFSM->Initializing State")
		    end,
	      doo=function()
		     while true do
			    raise_trigger_event("e_runTests")
			    rfsm.yield()
		     end
		  end,
	      exit=function()
		      --print("=>iTaSCFSM=>Running=>CompositeTaskFSM->Initialized State")
		      --print("===Application up and running!===")
		   end
	   },

	   Running = rfsm_ext.seqand {
	      order = {'CoordinatingTest','CompositeTestsFSMa'},
	      CoordinatingTest = rfsm.init(rfsm.load("scripts/running_test_coordination.lua")),
	      CompositeTestsFSMa = rfsm.init(rfsm.load("scripts/composite_test_fsm.lua")),
	   },

	   rfsm.transition { src='initial', tgt='Initializing' },
	   rfsm.transition { src='Initializing', tgt='Running'},
	},

	StoppingTest = rfsm.simple_state{
		entry = function ()
			--print("=>iTaSCFSM->StoppingITASC state entry")
		    lockRobots() 
            stopPr2connector()
            stopPr2Robot()
            stopTrajectoryGenerator()
            stopTestComponent()
			raise_trigger_event("e_stopTest")
		end,
	},

	StoppedTest = rfsm.simple_state{
		entry=function()
			--print("=>iTaSCFSM->StoppedITASC state")
			raise_common_event("e_TestStopped")
            killTest(0)
		end,
	},
	
	rfsm.transition { src='initial', tgt='PreOperational' },
	rfsm.transition { src='PreOperational', tgt='ConfiguringFirstPartTest', events={'e_configTest'}},
	rfsm.transition { src='ConfiguringFirstPartTest', tgt='ConfiguringPr2Connect', events={'e_FirstPartConfigured'}},
	rfsm.transition { src='ConfiguringPr2Connect', tgt='ConfiguringSecondPart',events={'e_Pr2Configured'}},
	rfsm.transition { src='ConfiguringPr2Connect', tgt='errorOnConfigure', events={ "e_after(5)" }},
	rfsm.transition { src='ConfiguringSecondPart', tgt='ConfiguredTest',events={'e_TestConfigured'}},
	rfsm.transition { src='ConfiguringSecondPart', tgt='StoppingTest', events={'e_stopTest'} },
	rfsm.transition { src='ConfiguredTest', tgt='StartingTest', events={'e_startUpTest'} },
	rfsm.transition { src='ConfiguredTest', tgt='StoppingTest', events={'e_stopTest'} },
	rfsm.transition { src='StartingTest', tgt='StartedTest', events={'e_startTest'} },
	rfsm.transition { src='StartingTest', tgt='StoppingTest', events={ 'e_stopTest' } },
	rfsm.transition { src='StartedTest', tgt='RunningTest', events={'e_runTests'} },
	rfsm.transition { src='StartedTest', tgt='StoppingTest', events={ 'e_stopTest' } },
	rfsm.transition { src='RunningTest', tgt='StoppingTest', events={ 'e_stopTest' } },
	rfsm.transition { src='StoppingTest', tgt='StoppedTest'	},
},

TestEmergency = rfsm.simple_state{
	entry = function ()
		--stopAllComponents()
        lockRobots() 
		raise_priority_event("e_emergency")
		--print("[EMERGENCY] =>iTaSC application in emergency state!")
		--print("[EMERGENCY] =>iTaSC has therefore raised an priority emergency event")
	end,
},

rfsm.transition { src='initial', tgt='NONemergency' },
rfsm.transition { src='NONemergency', tgt='TestEmergency', events={'e_emergency'} },
}
