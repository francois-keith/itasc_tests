
-- composite test fsm
-- Task Ids:
--   - move
--   - check poses
--   - check joint values


require "rfsm_ext"
require "rfsm_timeevent"
rfsm_timeevent.set_gettime_hook(rtt.getTime)

require "rfsmpp"

local state, trans = rfsm.state, rfsm.transition

-- the FSM
return state {
    dbg=rfsmpp.gen_dbgcolor("composite_test_fsm.lua", { STATE_ENTER=true, STATE_EXIT=true }, false),

	initializing = state{},

	Initialized = state{
		doo = function()
				startTrajectoryGenerator()
		end,
	},   


    -- Move
    move = state {
        entry=function ()
            	moveToNextPosition()    
	end

    },

    -- Wait till move is completed

    waitForMoveToFinish = state {
    },

    -- do checks.
    doChecks = state {
        doo=function ()
            doPoseChecks()
            raise_common_event('e_posechecks_completed')
        end
    },

    doJointChecks = state{
	doo=function ()
	    doJointValueChecks()
            raise_common_event('e_jointchecks_completed')
        end
    },

    testSucceeded = state{
        doo=function ()
            print(" !!! TEST SUCCEEDED !!! ")
            print("Quiting the test")
            raise_common_event('e_stopTest')
        end
    },

    -- checkError state
    checkError = state {
	doo=function ()
        print(" !!! TEST FAILED !!! ")
        print("Quiting the test")
        raise_common_event('e_stopTest')
	end
    },

    trans{ src='initial', tgt='initializing' },
    trans{src='initializing', tgt='Initialized'},
    rfsm.transition { src='Initialized', tgt='move'},
    trans{src = 'move', tgt = 'waitForMoveToFinish', events={"nAxes_generator_move_finished"}},
    trans{src = 'move', tgt = 'waitForMoveToFinish', events={"nAxes_generator_traj_finished"}},
    trans{src = 'waitForMoveToFinish', tgt = 'doJointChecks', events={'e_after(5)'}},
    trans{src='doChecks', tgt='checkError', events={'e_checkError'}},
    trans{src='doJointChecks', tgt='checkError', events={'e_checkError'}},
    trans{src='doJointChecks', tgt='doChecks', events={'e_jointchecks_completed'}},
    trans{src='doChecks', tgt='move', events={'e_posechecks_completed'}},
    trans{src='checkError', tgt='Initialized', events={'e_done'} },
    trans{src='move', tgt='testSucceeded', events={'e_test_done'} }
}
