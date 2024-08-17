//
// Created by caleb on 25/11/22.
//

#include "DARocketState.h"

using namespace std;

namespace JSBSim {

  DARocketState::DARocketState(FGFDMExec *exec) {
    FDMExec = exec;
    FDMExec->SetPropertyValue("propulsion/engine/rocket/transit-time", 0);
    PropertyManager = *exec->GetPropertyManager();
  }

  void DARocketState::BindStateTransitionTimes() {
    BindProperty("propulsion/engine/rocket/transition-time/safe-arm", safeToArmTime);
    BindProperty("propulsion/engine/rocket/transition-time/safe-dumping", safeToDumpingTime);
    BindProperty("propulsion/engine/rocket/transition-time/safe-filling", safeToFillingTime);
    BindProperty("propulsion/engine/rocket/transition-time/arm-monoprop", armToMonopropTime);
    BindProperty("propulsion/engine/rocket/transition-time/arm-safe", armToSafeTime);
    BindProperty("propulsion/engine/rocket/transition-time/monoprop-biprop", monopropToBipropTime);
    BindProperty("propulsion/engine/rocket/transition-time/monoprop-arm", monopropToArmTime);
    BindProperty("propulsion/engine/rocket/transition-time/monoprop-safe", monopropToSafeTime);
    BindProperty("propulsion/engine/rocket/transition-time/biprop-monoprop", bipropToMonopropTime);
    BindProperty("propulsion/engine/rocket/transition-time/dumping-safe", dumpingToSafeTime);
    BindProperty("propulsion/engine/rocket/transition-time/filling-safe", fillingToSafeTime);
    BindProperty("propulsion/engine/rocket/transition-time/system-arm-engine-arm", systemArmToEngineArmTime);
  }

  void DARocketState::BindProperty(const string& path, double &variable) {
    if (PropertyManager.HasNode(path)) {
      variable = FDMExec->GetPropertyValue(path);
    }
  }


  // if new state is -1 set to arm, this is a default value passed in by da-bridge
  // if next state is not the same as the current state check the decay and callbacks that may be in progress
  // else start process of moving to a new state by passing the new state
  // into the state function for the current rocket state
  void DARocketState::SetState(int newStateInt) {
    auto new_state = static_cast<eRocketStates>(newStateInt);
    if (nextState != state) {
      checkDecay();
      CheckCallback();
    } else {
      switch (state) {
        case SAFE:
          SafeState(new_state);
          break;
        case SYSTEM_ARM:
          SystemArmState(new_state);
          break;
        case ENGINE_ARM:
          EngineArmState(new_state);
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

  void DARocketState::SafeState(eRocketStates newState) {
    switch (newState) {
      case SYSTEM_ARM:
        AddCallback(newState, safeToArmTime);
        break;
      case DUMPING:
        AddCallback(newState, safeToDumpingTime);
        break;
      case FILLING:
        AddCallback(newState, safeToFillingTime);
        break;
      default:
        InvalidStateDebug(newState);
    }
  }

  void DARocketState::SystemArmState(eRocketStates newState) {
    switch (newState) {
      case SAFE:
        AddCallback(newState, armToSafeTime);
        break;
      case ENGINE_ARM:
        AddCallback(newState, systemArmToEngineArmTime);
        break;
      default:
        InvalidStateDebug(newState);
    }
  }

  void DARocketState::EngineArmState(eRocketStates newState) {
    switch (newState) {
      case SAFE:
        AddCallback(newState, armToSafeTime);
        break;
      case MONO_PROP:
        AddCallback(newState, armToMonopropTime);
        SetDecayState(MONO_PROP);
        break;
      case BI_PROP:
        AddCallback(newState, armToMonopropTime, MONO_PROP);
        SetDecayState(MONO_PROP);
        break;
      default:
        InvalidStateDebug(newState);
    }
  }

  void DARocketState::MonoPropState(eRocketStates newState) {
    switch (newState) {
      case SAFE:
        AddCallback(newState, monopropToSafeTime);
        SetDecayState(ENGINE_ARM);
        break;
      case ENGINE_ARM:
        AddCallback(newState, monopropToArmTime);
        SetDecayState(ENGINE_ARM);
        break;
      case BI_PROP:
        AddCallback(newState, monopropToBipropTime);
        SetDecayState(BI_PROP);
        break;
      default:
        InvalidStateDebug(newState);
    }
  }

  void DARocketState::BiPropState(eRocketStates newState) {
    switch (newState) {
      case SAFE:
        AddCallback(newState, bipropToMonopropTime, MONO_PROP);
        SetDecayState(MONO_PROP);
        break;
      case ENGINE_ARM:
        AddCallback(newState, bipropToMonopropTime, MONO_PROP);
        SetDecayState(MONO_PROP);
        break;
      case MONO_PROP:
        AddCallback(newState, bipropToMonopropTime);
        SetDecayState(MONO_PROP);
        break;
      default:
        InvalidStateDebug(newState);
    }
  }

  void DARocketState::DumpingState(eRocketStates newState) {
    switch (newState) {
      case SAFE:
        AddCallback(newState, dumpingToSafeTime);
        break;
      default:
        InvalidStateDebug(newState);
    }
  }

  void DARocketState::FillingState(eRocketStates newState) {
    switch (newState) {
      case SAFE:
        AddCallback(newState, fillingToSafeTime);
        break;
      default:
        InvalidStateDebug(newState);
    }
  }

  // Check if it is time to move into next state
  void DARocketState::CheckCallback() {
    if (FDMExec->GetSimTime() >= executeTime) {
      ArriveInState(nextState);
    }
  }

  // if in a decay state update time since decay started to adjust the isp and prop flow values
  void DARocketState::checkDecay() {
    if (decayState != NONE) {
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
  void DARocketState::AddCallback(eRocketStates _targetState, double time_delay, eRocketStates multi_hop_next_state) {
    targetState = _targetState;
    if (multi_hop_next_state == NO_STATE) {
      nextState = targetState;
    } else {
      nextState = multi_hop_next_state;
    }
    executeTime = FDMExec->GetSimTime() + time_delay;
    if (time_delay == 0) {
      CheckCallback();
    }
  }

  void DARocketState::ArriveInState(eRocketStates arrival_state) {
    // if not already in state then move into state
    if (arrival_state != state) {
      state = arrival_state;
      FDMExec->SetPropertyValue("propulsion/engine/rocket/decay-state", 0);
      FDMExec->SetPropertyValue("propulsion/fuel_dump", arrival_state == DUMPING);
      FDMExec->SetPropertyValue("propulsion/refuel", arrival_state == FILLING);

      decayState = NONE;
      FDMExec->SetPropertyValue("propulsion/engine/rocket/transit-time", 0);
      if (debug_lvl > 0) {
        cout << "ROCKET ARRIVED IN STATE: " << StateStrings[arrival_state] << ", at time: "
             << FDMExec->GetSimTime() << " seconds" << endl;
      }
    }

    if (nextState != targetState) {
      SetState(targetState);
    } else {
      ClearCallbacks();
    }
  }

  void DARocketState::SetDecayState(const eRocketStates &transitionState) {
    FDMExec->SetPropertyValue("propulsion/engine/rocket/transit-time", 0);
    stateChangeTime = FDMExec->GetPropertyValue("simulation/sim-time-sec");
    if (state == BI_PROP && transitionState == MONO_PROP) {
      decayState = BI_TO_MONO;
    } else if (state == MONO_PROP && transitionState == ENGINE_ARM) {
      decayState = MONO_TO_ARM;
    } else if (state == ENGINE_ARM && transitionState == MONO_PROP) {
      decayState = ARM_TO_MONO;
    } else if (state == MONO_PROP && transitionState == BI_PROP) {
      decayState = MONO_TO_BI;
    } else {
      decayState = NONE;
    }
  }

  void DARocketState::InvalidStateDebug(eRocketStates newState) {
    if (debug_lvl > 1 && state != newState) {
      cout << "INVALID STATE TRANSITION: " << StateStrings[state] << " -> " << StateStrings[newState] << ", at time: "
           << FDMExec->GetSimTime() << " seconds" << endl;
    }
  }

  void DARocketState::SetStartState(int startState) {
    if (debug_lvl > 0) {
      cout << "Start State set to: " << StateStrings[startState] << endl;
    }
    AddCallback(static_cast<eRocketStates>(startState), 0);
  }

  void DARocketState::Debug() {
    cout << "---------------------------------------------------------------" << endl;
    cout << "Current State: " << state << ":" << FDMExec->GetPropertyValue("propulsion/engine/rocket-state") << endl;
    cout << "Target State: " << targetState << endl;
    cout << "Decay State: " << decayState << ":" << FDMExec->GetPropertyValue("propulsion/engine/rocket/decay-state")
         << endl;
    cout << "Transit Time: " << FDMExec->GetPropertyValue("propulsion/engine/rocket/transit-time") << endl;
    cout << "Operation Mode: " << FDMExec->GetPropertyValue("propulsion/engine/operation-mode") << endl;
    cout << "ISP: " << FDMExec->GetPropertyValue("propulsion/engine/isp") << endl;
    cout << "Mixture ratio: " << FDMExec->GetPropertyValue("propulsion/engine/mixture-ratio") << endl;
    cout << "oxi flow: " << FDMExec->GetPropertyValue("propulsion/engine/oxi-flow-rate-pps") << endl;
    cout << "Total Impulse: " << FDMExec->GetPropertyValue("propulsion/engine/total-impulse") << endl;
    cout << "Total vac impulse: " << FDMExec->GetPropertyValue("propulsion/engine/total-vac-impulse") << endl;
    cout << "vacuum thrust lbs: " << FDMExec->GetPropertyValue("propulsion/engine/vacuum-thrust_lbs") << endl;
    cout << "Tank:0 Contents lbs: " << FDMExec->GetPropertyValue("propulsion/tank[0]/contents-lbs") << endl;
    cout << "Tank:1 Contents lbs: " << FDMExec->GetPropertyValue("propulsion/tank[1]/contents-lbs") << endl;
    cout << "---------------------------------------------------------------" << endl;
  }

}