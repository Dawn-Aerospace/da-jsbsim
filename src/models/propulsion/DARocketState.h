//
// Created by caleb on 25/11/22.
//

#ifndef JSB_STATE_ROCKETSTATE_H
#define JSB_STATE_ROCKETSTATE_H

#include "FGFDMExec.h"
#include <limits>

namespace JSBSim {

class FGFDMExec;
class FGPropertyManager;
enum eRocketStates { NO_STATE=-2, ENGINE_ARM=-1, MONO_PROP=0, BI_PROP=1, SAFE=2, FILLING=3, DUMPING=4, SYSTEM_ARM=5};

class DARocketState: FGJSBBase {
private:
    enum eRocketDecayState {NONE=0, BI_TO_MONO=1,MONO_TO_ARM=2, ARM_TO_MONO=3, MONO_TO_BI=4};
    eRocketDecayState decayState = NONE;
    double stateChangeTime = 0;
    map<int, string> StateStrings = {{-2,"NO_STATE"}, {-1, "ENGINE_ARM"},
      {0, "MONO_PROP"}, {1, "BI_PROP"}, {2, "SAFE"}, {3, "FILLING"},{4, "DUMPING"}, {5, "SYSTEM_ARM"}};
    eRocketStates state = SYSTEM_ARM;
    FGFDMExec* FDMExec;
    typedef void (DARocketState::*callback_function)(eRocketStates); // type for conciseness
    eRocketStates nextState = state;
    eRocketStates targetState = state;
    double executeTime = 0;
    FGPropertyManager PropertyManager;

    double safeToArmTime = 0;
    double safeToDumpingTime = 0;
    double safeToFillingTime = 0;
    double armToMonopropTime = 0;
    double armToSafeTime = 0;
    double monopropToBipropTime = 0;
    double monopropToArmTime = 0;
    double monopropToSafeTime = 0;
    double bipropToMonopropTime = 0;
    double dumpingToSafeTime = 0;
    double fillingToSafeTime = 0;
    double systemArmToEngineArmTime = 0;

    void SafeState(eRocketStates newState);
    void SystemArmState(eRocketStates newState);
    void EngineArmState(eRocketStates newState);
    void MonoPropState(eRocketStates newState);
    void BiPropState(eRocketStates newState);
    void DumpingState(eRocketStates newState);
    void FillingState(eRocketStates newState);
    void ClearCallbacks(){executeTime = std::numeric_limits<double>::infinity(); targetState=state; nextState=state;}
    void AddCallback(eRocketStates _targetState, double time_delay, eRocketStates multi_hop_next_state= NO_STATE);
    void ArriveInState(eRocketStates arrival_state);
    void CheckCallback();
    void SetDecayState(const eRocketStates &transitionState);
    void InvalidStateDebug(eRocketStates newState);
    void Debug();

    void BindProperty(const string& path, double &variable);




public:
    /** Constructor.
    @param exec pointer to JSBSim parent object, the FDM Executive.
    @param el a pointer to the XML Element instance representing the engine.*/

    DARocketState(FGFDMExec* exec);

    /** Destructor */
    ~DARocketState() override {};
    int GetState() const {return state;}
    int GetDecayState() const {return decayState;}
    void SetState(int newStateInt);
    void SetStartState(int startState);

    void BindStateTransitionTimes();

    void checkDecay();
};
}



#endif //JSB_STATE_ROCKETSTATE_H
