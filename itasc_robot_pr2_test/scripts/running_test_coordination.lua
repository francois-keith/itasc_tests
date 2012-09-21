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

-- running iTaSC coordination
require "time"
--require("ansicolors")

--local starttime={}
--local stoptime={}

return rfsm.composite_state{
	--dbg = fsmpp.gen_dbgcolor3({["STATE_ENTER"]=true, ["STATE_EXIT"]=true, ["HIBERNATING"]=false, ["EXEC_PATH"]=true, ["EFFECT"]=false, ["DOO"]=false, ["CHECKING"]=false, ["RAISED"]=true},  false, ansicolors.red ..  "CoordinatingITASC" .. ansicolors.reset ),

	initializing = rfsm.simple_state{
		entry=function()
			--print("=>iTaSCFSM=>Running=>Coordination-> initializing state entered")
            			--starttime.sec, starttime.nsec =rtt.getTime()
		end,
	},

	Initialized = rfsm.simple_state{
		entry=function()
				--print("entering iTaSCFSM=>Running=>Initialized   time: ", rtt.getTime())
		end,
	},

	iTaSCcoordPhase = rfsm.simple_state{
		entry=function()
				--t1={}
				--t2={}

				--stoptime.sec, stoptime.nsec =rtt.getTime()
				--print("======================================================>iTaSCFSM=>Running=>Coordination-> total loop time: " .. time.tostr_us(time.sub(stoptime,starttime)))
				--starttime.sec	= stoptime.sec
				--starttime.nsec	= stoptime.nsec

				--print("=>iTaSCFSM=>Running=>Coordination->  coordination Phase1 state entered @ ", rtt.getTime())

				--t1.sec, t1.nsec = rtt.getTime()
			updateRobot()
			passPositionToGen()
				--t2.sec, t2.nsec = rtt.getTime()
				--print("                                     coordination Phase1: time to execute 'pr2robotupdateRobots': " .. time.tostr_us(time.sub(t2,t1)))

				--t1.sec, t1.nsec = rtt.getTime()
				--t2.sec, t2.nsec = rtt.getTime()
				--print("                                     coordination Phase1: time to execute 'estim_pr2robotupdateRobots': " .. time.tostr_us(time.sub(t2,t1)))

				--t1.sec, t1.nsec = rtt.getTime()
			--SceneCalculatePoses()
				--t2.sec, t2.nsec = rtt.getTime()
				--print("                                     coordination Phase1: time to execute 'SceneCalculatePoses': " .. time.tostr_us(time.sub(t2,t1)))
				--print("trigger tasks")
			--raise_trigger_event("e_triggerTasks")
				--print('     triggered e_triggerTasks')
		
				--t1={}
				--t2={}

				--t1.sec, t1.nsec = rtt.getTime()
			--SceneCalculateA()
				--t2.sec, t2.nsec = rtt.getTime()
				--print("                                     coordination Phase2: time to execute 'SceneCalculateA': " .. time.tostr_us(time.sub(t2,t1)))

				--t1.sec, t1.nsec = rtt.getTime()
			--SceneSolvers_solve()
				--t2.sec, t2.nsec = rtt.getTime()
				--print("                                     coordination Phase2: time to execute 'SceneSolvers_solve': " .. time.tostr_us(time.sub(t2,t1)))

				--t1.sec, t1.nsec = rtt.getTime()
			--SceneHandOut()
				--t2.sec, t2.nsec = rtt.getTime()
				--print("                                     coordination Phase2: time to execute 'SceneHandOut': " .. time.tostr_us(time.sub(t2,t1)))

				--t1.sec, t1.nsec = rtt.getTime()
			sendToRobot()
				--t2.sec, t2.nsec = rtt.getTime()
				--print("                                     coordination Phase2: time to execute 'pr2robotSendToRobot': " .. time.tostr_us(time.sub(t2,t1)))

				--t1.sec, t1.nsec = rtt.getTime()
				--t2.sec, t2.nsec = rtt.getTime()

			raise_common_event('e_ITASCalgorithmDone')
		end,

	},

	rfsm.transition { src='initial', tgt='initializing' },
	rfsm.transition { src='initializing', tgt='Initialized' },
	rfsm.transition { src='Initialized', tgt='iTaSCcoordPhase'},
	rfsm.transition { src='iTaSCcoordPhase', tgt='Initialized', events={'e_done'} }, 
}
