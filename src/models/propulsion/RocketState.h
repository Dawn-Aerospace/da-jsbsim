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
enum eRocketStates { NO_STATE=-1, MONO_PROP=0, BI_PROP=1, SAFE=2, DUMPING=3, FILLING=4, ARM=5};

class RocketState: FGJSBBase {
private:
    enum eRocketDecayState {NONE=0, BI_TO_MONO=1,MONO_TO_ARM=2, ARM_TO_MONO=3, MONO_TO_BI=4};
    eRocketDecayState decayState = NONE;
    double stateChangeTime = 0;
    string StateStrings[6] = {"MONO_PROP", "BI_PROP", "SAFE","DUMPING", "FILLING", "ARM"};
    eRocketStates state = SAFE;
    FGFDMExec* FDMExec;
    typedef void (RocketState::*callback_function)(eRocketStates); // type for conciseness
    eRocketStates nextState = state;
    eRocketStates targetState = state;
    double executeTime = 0;
    void SafeState(eRocketStates newState);
    void ArmState(eRocketStates newState);
    void MonoPropState(eRocketStates newState);
    void BiPropState(eRocketStates newState);
    void DumpingState(eRocketStates newState);
    void FillingState(eRocketStates newState);
    void clearCallbacks(){executeTime = std::numeric_limits<double>::infinity(); targetState=state; nextState=state;}
    void addCallback(eRocketStates _targetState, double time_delay, eRocketStates multi_hop_next_state=NO_STATE);
    void arriveInState(eRocketStates arrival_state);
    void checkCallback();
    void setDecayState(const eRocketStates &functionState);
    void invalidStateDebug(eRocketStates newState);
    void Debug();


public:
    /** Constructor.
    @param exec pointer to JSBSim parent object, the FDM Executive.
    @param el a pointer to the XML Element instance representing the engine.*/

    RocketState(FGFDMExec* exec);

    /** Destructor */
    ~RocketState() override {};
    int GetState() const {return state;}
    int GetDecayState() const {return decayState;}
    void SetState(int newStateInt);
    void SetStartState(int startState);

    void checkDecay();
};
}



#endif //JSB_STATE_ROCKETSTATE_H
