//
// Created by caleb on 25/11/22.
//

#include "RocketState.h"

using namespace std;

namespace JSBSim {

    RocketState::RocketState(FGFDMExec* exec) {
        FDMExec = exec;
        FDMExec->SetPropertyValue("propulsion/engine/rocket/transit-time", 0);
        // Set the starting state of the rocket to the value written in the xml
        FDMExec->SetPropertyValue("propulsion/engine/rocket/start-state", FDMExec->GetPropertyValue("propulsion/engine/rocket/initial_state"));
    }

    // if new state is - set to arm, this is a default value passed in by da-bridge
    // if next state is not the same as the current state check the decay and callbacks that may be in progress
    // else start process of moving to a new state by passing the new state
    // into the state function for the current rocket state
    void RocketState::SetState(int newStateInt) {
        if (newStateInt == -1) {
            newStateInt = ARM;
        }
        auto new_state = static_cast<eRocketStates>(newStateInt);
        if (nextState != state) {
            checkDecay();
            checkCallback();
        } else {
            switch (state) {
                case SAFE:
                    SafeState(new_state);
                    break;
                case ARM:
                    ArmState(new_state);
                    break;
                case MONO_PROP:
                    MonoPropState(new_state);
                    break;
                case BI_PROP:
                    BiPropState(new_state);
                    break;
                case DUMPING:
                    DumpingState(new_state);
                    break;
                case FILLING:
                    FillingState(new_state);
                    break;
            }
        }
    }

    void RocketState::SafeState(eRocketStates newState) {
        switch (newState){
            case ARM:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/safe-arm"));
                break;
            case DUMPING:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/safe-dumping"));
                break;
            case FILLING:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/safe-filling"));
                break;
            default:
                invalidStateDebug(newState);
        }
    }

    void RocketState::ArmState(eRocketStates newState) {
        switch (newState){
            case SAFE:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/arm-safe"));
                break;
            case MONO_PROP:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/arm-monoprop"));
                setDecayState(MONO_PROP);
                break;
            case BI_PROP:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/arm-monoprop"), MONO_PROP);
                setDecayState(MONO_PROP);
                break;
            default:
                invalidStateDebug(newState);
        }
    }

    void RocketState::MonoPropState(eRocketStates newState) {
        switch (newState){
            case SAFE:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/monoprop-safe"));
                //setDecayState(ARM);
                break;
            case ARM:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/monoprop-arm"));
                setDecayState(ARM);
                break;

            case BI_PROP:
                addCallback(newState,
                            FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/monoprop-biprop"));
                setDecayState(BI_PROP);
                break;
            default:
                invalidStateDebug(newState);
        }
    }

    void RocketState::BiPropState(eRocketStates newState) {
        switch (newState){
            case SAFE:
                addCallback(newState, FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/biprop-safe"));
                //setDecayState(MONO_PROP);
                break;
            case ARM:
                addCallback(newState, FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/biprop-monoprop"), MONO_PROP);
                setDecayState(MONO_PROP);
                break;
            case MONO_PROP:
                addCallback(newState, FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/biprop-monoprop"));
                setDecayState(MONO_PROP);
                break;
            default:
                invalidStateDebug(newState);
        }
    }

    void RocketState::DumpingState(eRocketStates newState) {
        switch (newState){
            case SAFE:
                addCallback(newState, FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/dumping-safe"));
                break;
            default:
                invalidStateDebug(newState);
        }
    }

    void RocketState::FillingState(eRocketStates newState) {
        switch (newState){
            case SAFE:
                addCallback(newState, FDMExec->GetPropertyValue("propulsion/engine/rocket/transition-time/filling-safe"));
                break;
            default:
                invalidStateDebug(newState);
        }
    }

    // Check if it is time to move into next state
    void RocketState::checkCallback() {
        if (FDMExec->GetSimTime() >= executeTime) {
            arriveInState(nextState);
        }
    }

    // if in a decay state update time since decay started to adjust the isp and prop flow values
    void RocketState::checkDecay() {
        if(decayState != NONE) {
            double timeSinceStateChange = FDMExec->GetPropertyValue("simulation/sim-time-sec") - stateChangeTime;
            FDMExec->SetPropertyValue("propulsion/engine/rocket/transit-time", timeSinceStateChange);
        }
    }

    /* set target state execute time of callback function and if time is zero call it immediately
     @param _targetState = the state that is being transitioned to
     @param time_delay = how long to wait before moving into the next state
     @param multi_hop_next_state = optional parameter used if a state transition requires more than 1 hop,
            ie arm to biprop via monoprop, biprop is _targetState and monoprop is multi_hop_next_state
     */
    void RocketState::addCallback(eRocketStates _targetState, double time_delay, eRocketStates multi_hop_next_state) {
        targetState = _targetState;
        if (multi_hop_next_state == NO_STATE) {
            nextState = targetState;
        } else {
            nextState = multi_hop_next_state;
        }
        executeTime = FDMExec->GetSimTime() + time_delay;
        if (time_delay == 0) {
            checkCallback();
        }
    }

    void RocketState::arriveInState(eRocketStates arrival_state) {
        // if not already in state then move into state
        if (arrival_state != state){
            state = arrival_state;
            FDMExec->SetPropertyValue("propulsion/engine/rocket/decay-state", 0);
            FDMExec->SetPropertyValue("propulsion/fuel_dump", arrival_state == DUMPING);
            FDMExec->SetPropertyValue("propulsion/refuel", arrival_state == FILLING);
            FDMExec->SetPropertyValue("fcs/engine-opmode[0]", arrival_state);
            FDMExec->SetPropertyValue("fcs/engine-opmode-cmd[0]", arrival_state);

            decayState = NONE;
            FDMExec->SetPropertyValue("propulsion/engine/rocket/transit-time", 0);
            if (debug_lvl > 0) {
                cout << "ROCKET ARRIVED IN STATE: " << StateStrings[arrival_state] << ", at time: "
                     << FDMExec->GetSimTime() << " seconds" << endl;
            }
        }
        //do we need to call set state this causes the recursion
        if (nextState != targetState) {
            SetState(targetState);
        } else {
            clearCallbacks();
        }
    }

    void RocketState::setDecayState(const eRocketStates &functionState) {
        FDMExec->SetPropertyValue("propulsion/engine/rocket/transit-time", 0);
        FDMExec->SetPropertyValue("fcs/engine-opmode[0]", nextState);
        FDMExec->SetPropertyValue("fcs/engine-opmode-cmd[0]", nextState);
        stateChangeTime = FDMExec->GetPropertyValue("simulation/sim-time-sec");
        if (state == BI_PROP && functionState == MONO_PROP) {
            decayState = BI_TO_MONO;
        } else if (state == MONO_PROP && functionState == ARM) {
            decayState = MONO_TO_ARM;
        } else if (state == ARM && functionState == MONO_PROP) {
            decayState = ARM_TO_MONO;
        } else if (state == MONO_PROP && functionState == BI_PROP) {
            decayState = MONO_TO_BI;
        } else {
            decayState = NONE;
        }
    }

    void RocketState::invalidStateDebug(eRocketStates newState) {
        if (debug_lvl > 0 && state != newState) {
            cout<<"INVALID STATE TRANSITION: "<< StateStrings[state]<< " -> " << StateStrings[newState] <<", at time: "
                << FDMExec->GetSimTime() << " seconds" <<endl;
        }
    }

    void RocketState::SetStartState(int startState) {
        if (debug_lvl > 0) {
            cout << "Start State set to: " << StateStrings[startState] << endl;
        }
        addCallback(static_cast<eRocketStates>(startState),0);
    }

    void RocketState::Debug() {
        cout<<"---------------------------------------------------------------"<<endl;
        cout << "Current State: " << state<<":"<<FDMExec->GetPropertyValue("propulsion/engine/rocket-state") <<endl;
        cout << "Target State: "<<targetState<<endl;
        cout << "Decay State: "<<decayState<<":"<<FDMExec->GetPropertyValue("propulsion/engine/rocket/decay-state") <<endl;
        cout << "Transit Time: "<<FDMExec->GetPropertyValue("propulsion/engine/rocket/transit-time")<<endl;
        cout << "Operation Mode: "<<FDMExec->GetPropertyValue("propulsion/engine/operation-mode")<<endl;
        cout << "ISP: "<<FDMExec->GetPropertyValue("propulsion/engine/isp")<<endl;
        cout << "Mixture ratio: " <<FDMExec->GetPropertyValue("propulsion/engine/mixture-ratio")<<endl;
        cout << "oxi flow: " <<FDMExec->GetPropertyValue("propulsion/engine/oxi-flow-rate-pps")<<endl;
        cout << "Total Impulse: "<<FDMExec->GetPropertyValue("propulsion/engine/total-impulse")<<endl;
        cout << "Total vac impulse: " <<FDMExec->GetPropertyValue("propulsion/engine/total-vac-impulse")<<endl;
        cout << "vacuum thrust lbs: " <<FDMExec->GetPropertyValue("propulsion/engine/vacuum-thrust_lbs")<<endl;
        cout << "Tank:0 Contents lbs: " <<FDMExec->GetPropertyValue("propulsion/tank[0]/contents-lbs")<<endl;
        cout << "Tank:1 Contents lbs: " <<FDMExec->GetPropertyValue("propulsion/tank[1]/contents-lbs")<<endl;
        cout<<"---------------------------------------------------------------"<<endl;
    }

}