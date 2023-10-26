//
// Created by caleb on 1/02/23.
//

#ifndef JSBSIM_DAWALLTEMPESTIMATIONTEST_H
#define JSBSIM_DAWALLTEMPESTIMATIONTEST_H

#include <cxxtest/TestSuite.h>
#include "FGFDMExec.h"
#include "models/atmosphere/FGStandardAtmosphere.h"
#include "models/DAWallTempEstimation.h"


using namespace JSBSim;


class DAWallTempEstimationTest : public CxxTest::TestSuite, public JSBSim::DAWallTempEstimation{
private:
    JSBSim::FGFDMExec FDMExec;
    double _airTemp_K;
    double _airPressure_Pa;
    double _kinematicViscosity;
    double _absoluteViscosity;
    double _soundSpeed_ms;
    int _maxIterations=30;
    
    double METER_TO_FEET_ = 1/0.3048;
    double KELVIN_TO_RANKINE_ = 1.8;
    double PASCAL_TO_PSF_ = 1/47.880208;

    double FEET_TO_METER_ = 0.3048;
    double RANKINE_TO_KELVIN_ = 1/1.8;
    double PSF_TO_PASCAL_ = 47.880208;


    void setupTest() {
      DAWallTempEstimation();
      MockAtmosphere mockAtmosphere = MockAtmosphere(&FDMExec);
      maxIterations=_maxIterations;
      mockAtmosphere.SetTemperature(_airTemp_K);
      mockAtmosphere.SetPressure(_airPressure_Pa);
      mockAtmosphere.SetKinematicViscosity(_kinematicViscosity);
      mockAtmosphere.SetAbsoluteViscosity(_absoluteViscosity);
      mockAtmosphere.SetSoundSpeed(_soundSpeed_ms);
      atmosphere_ = std::make_shared<MockAtmosphere>(mockAtmosphere);
    }

    class MockAtmosphere: public FGStandardAtmosphere {
    public:
        double airTemp_K;
        double airPressure_Pa;
        double kinematicViscosity;
        double absoluteViscosity;
        double soundSpeed_ms;
        MockAtmosphere(FGFDMExec* fdmex)
                : FGStandardAtmosphere(fdmex){};
        virtual double GetTemperature() const override {return airTemp_K;}
        virtual double GetPressure() const override {return airPressure_Pa;}
        virtual double GetKinematicViscosity() const override {return kinematicViscosity;}
        virtual double GetAbsoluteViscosity() const override {return absoluteViscosity;}
        virtual double GetSoundSpeed() const override {return soundSpeed_ms;}
        void SetTemperature(double value) {airTemp_K = value;}
        void SetPressure(double value) {airPressure_Pa = value;}
        void SetKinematicViscosity(double value) { kinematicViscosity = value;}
        void SetAbsoluteViscosity(double value) {absoluteViscosity = value;}
        void SetSoundSpeed(double value) {soundSpeed_ms = value;}
    };

public:

    void testMockAtmosphereOverride() {
      _airTemp_K = 5;
      setupTest();
      TS_ASSERT_EQUALS(atmosphere_->GetTemperature(), 5);
    }

    // Mach 1 at 10k turbulent flow
    void testGetWallTempEstimateCelsiusMach1Alt10k() {
      _airTemp_K = 223.14999999999998 * KELVIN_TO_RANKINE_;
      _airPressure_Pa = 388349.24720240704 * PASCAL_TO_PSF_;
      _kinematicViscosity = 2.4034574673532144e-06;
      _absoluteViscosity = 1.45710858090486e-05;
      _soundSpeed_ms = 299.4659948926586 * METER_TO_FEET_;
      setupTest();
      flowType_ = 1;
      TS_ASSERT_EQUALS(round(GetWallTempEstimateCelsius(1 * METER_TO_FEET_,1)), -10);
    }

    // Mach 1 at 10k laminar flow
    void testGetWallTempEstimateCelsiusMach1Alt10kLaminarFlow() {
      _airTemp_K = 223.14999999999998 * KELVIN_TO_RANKINE_;
      _airPressure_Pa = 388349.24720240704 * PASCAL_TO_PSF_;
      _kinematicViscosity = 2.4034574673532144e-06;
      _absoluteViscosity = 1.45710858090486e-05;
      _soundSpeed_ms = 299.4659948926586 * METER_TO_FEET_;
      setupTest();
      flowType_ = 0;
      TS_ASSERT_EQUALS(round(GetWallTempEstimateCelsius(1 * METER_TO_FEET_,1)), -12);
    }

    // Mach 2.04 at 18.3k turbulent flow
    void testGetWallTempEstimateCelsiusMach2Alt18k() {
      _airTemp_K = 216.65 * KELVIN_TO_RANKINE_;
      _airPressure_Pa = 7158.118362641401 * PASCAL_TO_PSF_;
      _kinematicViscosity = 0.00012351261888837405;
      _absoluteViscosity = 1.4216130796413358e-05;
      _soundSpeed_ms = 295.0722820056179 * METER_TO_FEET_;
      setupTest();
      flowType_ = 1;
      TS_ASSERT_EQUALS(round(GetWallTempEstimateCelsius(1 * METER_TO_FEET_,2.04)), 106);
    }

    // Mach 3.1 at 25k turbulent flow
    void testGetWallTempEstimateCelsiusMach3Alt25k() {
      _airTemp_K = 221.65 * KELVIN_TO_RANKINE_;
      _airPressure_Pa = 11936.585391666198 * PASCAL_TO_PSF_;
      _kinematicViscosity = 7.723486130952107e-05;
      _absoluteViscosity = 1.4489574855925882e-05;
      _soundSpeed_ms = 298.4578021705926 * METER_TO_FEET_;
      setupTest();
      flowType_ = 1;
      TS_ASSERT_EQUALS(round(GetWallTempEstimateCelsius(1 * METER_TO_FEET_,3.1)), 326);
    }

    // Mach 4 at 30k turbulent flow
    void testGetWallTempEstimateCelsiusMach4Alt30k() {
      _airTemp_K = 226.65 * KELVIN_TO_RANKINE_;
      _airPressure_Pa = 25576.828953634504 * PASCAL_TO_PSF_;
      _kinematicViscosity = 3.7547057144736505e-05;
      _absoluteViscosity = 1.47603541438064e-05;
      _soundSpeed_ms = 301.8053474426823 * METER_TO_FEET_;
      setupTest();
      flowType_ = 1;
      TS_ASSERT_EQUALS(round(GetWallTempEstimateCelsius(1 * METER_TO_FEET_,4.0)), 579);
    }

    //Mach speed of 0 breaks calc with zero division, 0.0001Ma = 0.12348kph
    void testMinMachSpeed() {
      setupTest();
      maxIterations=0;
      GetWallTempEstimateCelsius(1,0);
      TS_ASSERT_EQUALS(machSpeed, 0.0001);
    }

    void testRegularMachSpeed() {
      setupTest();
      maxIterations=0;
      GetWallTempEstimateCelsius(1,1.4);
      TS_ASSERT_EQUALS(machSpeed, 1.4);
    }

    void testChordConversion() {
      setupTest();
      maxIterations=0;
      GetWallTempEstimateCelsius(1 * METER_TO_FEET_,1);
      TS_ASSERT_EQUALS(round(chord_m), 1.0);
    }

    void testSetInputs() {
      _airTemp_K = 216.0 * KELVIN_TO_RANKINE_;
      _airPressure_Pa = 7158.0 * PASCAL_TO_PSF_;
      _kinematicViscosity = 0.0001;
      _absoluteViscosity = 1.4e-05;
      _soundSpeed_ms = 295.0 * FEET_TO_METER_;
      setupTest();
      SetInputs();
      TS_ASSERT_EQUALS(airTemp_K, _airTemp_K * RANKINE_TO_KELVIN_);
      TS_ASSERT_EQUALS(airPressure_Pa, _airPressure_Pa * PSF_TO_PASCAL_);
      TS_ASSERT_EQUALS(kinematicViscosity, _kinematicViscosity);
      TS_ASSERT_EQUALS(absoluteViscosity, _absoluteViscosity);
      TS_ASSERT_EQUALS(soundSpeed_ms, _soundSpeed_ms * FEET_TO_METER_);
    }

    void testCalculateCpAndGammaWithTempBelow250() {
      double cp;
      double gm;
      tie(cp, gm) = CalculateCpAndGamma(250);
      TS_ASSERT_EQUALS(round(cp), 1003);
      TS_ASSERT_EQUALS(gm, 1.003/0.716);
    }

    void testCalculateCpAndGammaWithTempInRange() {
      double cp;
      double gm;
      tie(cp, gm) = CalculateCpAndGamma(600);
      TS_ASSERT_EQUALS(cp, 1051);
      TS_ASSERT_EQUALS(gm, 1.051/0.764);
    }

    void testCalculateCpAndGammaWithTempAbove1500() {
      double cp;
      double gm;
      tie(cp, gm) = CalculateCpAndGamma(1600);
      TS_ASSERT_EQUALS(cp, 1216);
      TS_ASSERT_EQUALS(gm, 1.216/0.929);
    }
};

#endif //JSBSIM_DAWALLTEMPESTIMATIONTEST_H
