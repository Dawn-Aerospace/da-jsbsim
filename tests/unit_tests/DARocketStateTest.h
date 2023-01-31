//
// Created by caleb on 29/11/22.
//

#ifndef JSB_STATE_ROCKETSTATETEST_H
#define JSB_STATE_ROCKETSTATETEST_H

#include <cxxtest/TestSuite.h>
#include <FGFDMExec.h>
#include "initialization/FGInitialCondition.h"

using namespace JSBSim;


class DARocketStateTest : public CxxTest::TestSuite {
private:
    enum eRocketStates { NO_STATE=-2, ENGINE_ARM=-1, MONO_PROP=0, BI_PROP=1, SAFE=2, FILLING=3, DUMPING=4, SYSTEM_ARM=5};
    FGFDMExec setupTest() {
      std::string aircraft_path_ = "../../../aircraft";
      std::string model_name_ = "RocketPlane";
      std::string scene_path_ = "reset00.xml";
      double simulation_rate_ = 0.008333333333333333;
      JSBSim::FGFDMExec FDMExec;
      FDMExec.SetDebugLevel(0);
      FDMExec.SetAircraftPath(SGPath(aircraft_path_));
      auto IC_ = FDMExec.GetIC();
      FDMExec.LoadModel(model_name_);
      IC_->Load(SGPath(scene_path_));
      FDMExec.Setdt(simulation_rate_);
      FDMExec.RunIC();
      return FDMExec;
    }

    FGFDMExec setupTestWithStartState(int startState) {
      std::string aircraft_path_ = "../../../aircraft";
      std::string model_name_ = "RocketPlane";
      std::string scene_path_ = "reset00.xml";
      double simulation_rate_ = 0.008333333333333333;
      JSBSim::FGFDMExec FDMExec;
      FDMExec.SetDebugLevel(0);
      FDMExec.SetAircraftPath(SGPath(aircraft_path_));
      auto IC_ = FDMExec.GetIC();
      FDMExec.LoadModel(model_name_);
      IC_->Load(SGPath(scene_path_));
      FDMExec.Setdt(simulation_rate_);
      FDMExec.RunIC();
      FDMExec.SetPropertyValue("propulsion/engine/rocket/start-state", startState);
      return FDMExec;
    }

public:

    void testStartingState() {
      FGFDMExec FDMExec = setupTest();
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), SAFE);
    }

    void testStartStateWhenSetToBiProp() {
      FGFDMExec FDMExec = setupTestWithStartState(1);
      while (FDMExec.GetSimTime() < 5) {
        FDMExec.Run();
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), BI_PROP);
    }

    void testSafeToArmTransition() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 5) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 5);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), SYSTEM_ARM);
    }

    void testSafeToMonoPropTransition() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 1) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", SYSTEM_ARM);
        } else if (1 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", ENGINE_ARM);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", MONO_PROP);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), MONO_PROP);
    }

    void TestSafeToBiPropDirect() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 15) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", SYSTEM_ARM);
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", ENGINE_ARM);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 15) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", BI_PROP);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), BI_PROP);
    }

    void TestSafeToBiPropViaMono() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 15) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 1) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", SYSTEM_ARM);
        } else if (1 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", ENGINE_ARM);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 9) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", MONO_PROP);
        } else if (9 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 15) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", BI_PROP);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), BI_PROP);
    }

    void TestBiPropToArm() {
      FGFDMExec FDMExec = setupTestWithStartState(BI_PROP);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", MONO_PROP);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 8) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", ENGINE_ARM);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), ENGINE_ARM);
    }

    void testFuelDumping() {
      FGFDMExec FDMExec = setupTest();
      double initial_fuel = FDMExec.GetPropertyValue("propulsion/tank[0]/contents-lbs");
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.SetPropertyValue("propulsion/engine/rocket-state", DUMPING);
        FDMExec.Run();
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/fuel_dump"), BI_PROP);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/refuel"), MONO_PROP);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), DUMPING);
      TS_ASSERT_LESS_THAN(FDMExec.GetPropertyValue("propulsion/tank[0]/contents-lbs"), initial_fuel);
    }

    void testFilling() {
      FGFDMExec FDMExec = setupTestWithStartState(DUMPING);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
      }
      while (FDMExec.GetSimTime() < 12) {
        FDMExec.SetPropertyValue("propulsion/engine/rocket-state", SAFE);
        FDMExec.Run();
      }
      double low_fuel = FDMExec.GetPropertyValue("propulsion/tank[0]/contents-lbs");
      while (FDMExec.GetSimTime() < 15) {
      FDMExec.SetPropertyValue("propulsion/engine/rocket-state", FILLING);
        FDMExec.Run();
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/fuel_dump"), MONO_PROP);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/refuel"), BI_PROP);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), FILLING);
      TS_ASSERT_LESS_THAN(low_fuel, FDMExec.GetPropertyValue("propulsion/tank[0]/contents-lbs"));
    }

    void testCantStateOverride() {
      FGFDMExec FDMExec = setupTest();
      FDMExec.SetPropertyValue("propulsion/engine/rocket-state", SYSTEM_ARM);
      while (FDMExec.GetSimTime() < 5) {
        FDMExec.Run();
        FDMExec.SetPropertyValue("propulsion/engine/rocket-state", DUMPING);
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), SYSTEM_ARM);
    }

    void testInvalidSafeTransitions() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", MONO_PROP);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", BI_PROP);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), SAFE);
    }

    void testInvalidArmTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(SYSTEM_ARM);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), SYSTEM_ARM);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", FILLING);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", DUMPING);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), SYSTEM_ARM);
    }

    void testInvalidMonoPropTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(MONO_PROP);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", DUMPING);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", FILLING);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), MONO_PROP);
    }


    void testInvalidBiPropTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(BI_PROP);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", DUMPING);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", FILLING);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), BI_PROP);
    }

    void testInvalidDumpingTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(DUMPING);
      while (FDMExec.GetSimTime() < 20) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", MONO_PROP);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", BI_PROP);
        } else if (10 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 15) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", SYSTEM_ARM);
        } else if (15 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 20) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", FILLING);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), DUMPING);
    }

    void testInvalidFillingTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(FILLING);
      while (FDMExec.GetSimTime() < 2) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", MONO_PROP);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", BI_PROP);
        } else if (10 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 15) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", SYSTEM_ARM);
        } else if (15 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 20) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", DUMPING);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), FILLING);
    }

};

#endif //JSB_STATE_ROCKETSTATETEST_H
