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
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 2);
    }

    void testStartStateWhenSetToBiProp() {
      FGFDMExec FDMExec = setupTestWithStartState(1);
      while (FDMExec.GetSimTime() < 5) {
        FDMExec.Run();
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 1);
    }

    void testSafeToArmTransition() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 5) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 5);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 5);
    }

    void testSafeToMonoPropTransition() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 5);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 0);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 0);

    }

    void TestSafeToBiPropDirect() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 15) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 5);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 15) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 1);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 1);
    }

    void TestSafeToBiPropViaMono() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 15) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 3) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 5);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 9) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 0);
        } else if (9 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 15) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 1);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 1);
    }

    void TestBiPropToArm() {
      FGFDMExec FDMExec = setupTestWithStartState(1);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 0);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 5);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 5);
    }

    void testFuelDumping() {
      FGFDMExec FDMExec = setupTest();
      double initial_fuel = FDMExec.GetPropertyValue("propulsion/tank[0]/contents-lbs");
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 3);
        FDMExec.Run();
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/fuel_dump"), 1);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/refuel"), 0);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 3);
      TS_ASSERT_LESS_THAN(FDMExec.GetPropertyValue("propulsion/tank[0]/contents-lbs"), initial_fuel);
    }

    void testFilling() {
      FGFDMExec FDMExec = setupTestWithStartState(3);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
      }
      while (FDMExec.GetSimTime() < 12) {
        FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 2);
        FDMExec.Run();
      }
      double low_fuel = FDMExec.GetPropertyValue("propulsion/tank[0]/contents-lbs");
      while (FDMExec.GetSimTime() < 15) {
        FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 4);
        FDMExec.Run();
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/fuel_dump"), 0);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/refuel"), 1);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 4);
      TS_ASSERT_LESS_THAN(low_fuel, FDMExec.GetPropertyValue("propulsion/tank[0]/contents-lbs"));
    }

    void testCantStateOverride() {
      FGFDMExec FDMExec = setupTest();
      FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 5);
      while (FDMExec.GetSimTime() < 5) {
        FDMExec.Run();
        FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 3);
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 5);
    }

    void testInvalidSafeTransitions() {
      FGFDMExec FDMExec = setupTest();
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 0);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 1);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 2);
    }

    void testInvalidArmTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(5);
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 5);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 3);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 4);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 5);
    }

    void testInvalidMonoPropTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(0);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 3);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 4);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 0);
    }


    void testInvalidBiPropTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(1);
      while (FDMExec.GetSimTime() < 10) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 3);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 4);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 1);
    }

    void testInvalidDumpingTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(3);
      while (FDMExec.GetSimTime() < 20) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 0);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 1);
        } else if (10 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 15) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 5);
        } else if (15 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 20) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 4);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 3);
    }

    void testInvalidFillingTransitions() {
      FGFDMExec FDMExec = setupTestWithStartState(4);
      while (FDMExec.GetSimTime() < 2) {
        FDMExec.Run();
        if (0 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 5) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 0);
        } else if (5 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 10) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 1);
        } else if (10 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 15) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 1);
        } else if (15 < FDMExec.GetSimTime() && FDMExec.GetSimTime() < 20) {
          FDMExec.SetPropertyValue("propulsion/engine/rocket-state", 3);
        }
      }
      TS_ASSERT_EQUALS(FDMExec.GetPropertyValue("propulsion/engine/rocket-state"), 4);
    }

};

#endif //JSB_STATE_ROCKETSTATETEST_H
