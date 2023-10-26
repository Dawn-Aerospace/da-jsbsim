//
// Created by caleb on 23/01/23.
//
#include "FGFDMExec.h"
#include "DAWallTempEstimation.h"


namespace JSBSim {

    DAWallTempEstimation::DAWallTempEstimation(FGFDMExec* _fdmex){
      fdmex_ = _fdmex;
      atmosphere_ = fdmex_->GetAtmosphere();
    }

    double DAWallTempEstimation::GetWallTempEstimateCelsius(double _chord, double _machSpeed) {
      chord_m = _chord * FEET_TO_METER_;
      machSpeed = _machSpeed < 0.0001 ? 0.0001 : _machSpeed;
      SetInputs();
      velocity_ms = machSpeed * soundSpeed_ms;
      reynoldsNumber = velocity_ms * chord_m / kinematicViscosity;
      if (flowType_ == 0) {
        C = 0.664;
        N = 0.5;
        a = 0.032;
        b = 0.58;
      }
      else {
        C = 0.0592;
        N = 0.2;
        a = 0.035;
        b = 0.45;
      }
      cfi = C / (pow(reynoldsNumber, N));
      NN = 1 - N * (w + 1);
      return DAWallTempEstimation::NewtonRaphson() + KELVIN_TO_CELSIUS_;
    }

    void DAWallTempEstimation::SetInputs() {
      airTemp_K = atmosphere_->GetTemperature() * RANKINE_TO_KELVIN_;
      airPressure_Pa = atmosphere_->GetPressure() * PSF_TO_PASCAL_;
      kinematicViscosity = atmosphere_->GetKinematicViscosity();
      absoluteViscosity = atmosphere_->GetAbsoluteViscosity();
      soundSpeed_ms = atmosphere_->GetSoundSpeed() * FEET_TO_METER_;
    }

    double DAWallTempEstimation::HeatBalance(double estimate) const {
      double cp;
      double gm;
      tie(cp, gm) = CalculateCpAndGamma(airTemp_K);
      double const Tref = 1.0 + a * pow(machSpeed, 2.0) + b * (estimate / airTemp_K - 1.0);
      double const Pr = absoluteViscosity * cp * (airTemp_K + 245.0 * pow(10.0, (-12.0 / airTemp_K)))/ (2.64638e-3 * pow(airTemp_K, 1.5));
      double const q = 0.5 * gm * airPressure_Pa * pow(machSpeed, 2.0);
      double const cf = cfi / (pow(Tref, NN));
      double const r = flowType_ == 0? pow(Pr, 0.5) : pow(Pr, (1.0 / 3.0));
      double const St = 0.5 * cf / pow(Pr, (2.0 / 3.0));
      double const Twad = airTemp_K * (1.0 + 0.5 * r * (gm - 1.0) * pow(machSpeed, 2.0));
      double const conducted = St * r * q * velocity_ms * (Twad - estimate) / (Twad - airTemp_K);
      double radiated = 0;
      if (machSpeed > 1.4) {
        radiated = emissivity_ * sg * pow((estimate - airTemp_K), 4.0);
      }
      double const balance = conducted - radiated;
      return balance;
    }

    std::tuple<double, double> DAWallTempEstimation::CalculateCpAndGamma(double temperature) {
      vector<int> T_arr = {250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500};
      vector<double> cp_arr = {1.003, 1.005, 1.008, 1.013, 1.02, 1.029, 1.04, 1.051, 1.063, 1.075, 1.087, 1.099, 1.121, 1.142, 1.155,
              1.173, 1.19, 1.204, 1.216};
      vector<double> cv_arr = {0.716, 0.718, 0.721, 0.726, 0.733, 0.742, 0.753, 0.764, 0.776, 0.788, 0.8, 0.812, 0.834, 0.855, 0.868,
              0.886, 0.903, 0.917, 0.929};
      double cp;
      double cv;
      int i = -1;
      for (int n =0; n< T_arr.size(); n++) {
        if (temperature < T_arr[n]) {
          i = n;
          break;
        }
      }
      if (i == -1) {
        cp = cp_arr[18];
        cv = cv_arr[18];
      }else if (i == 0){
        cp = cp_arr[0];
        cv = cv_arr[0];
      } else {
        double const fraction = (temperature - T_arr[i - 1]) / (T_arr[i] - T_arr[i - 1]);
        cp = cp_arr[i - 1] + fraction * (cp_arr[i] - cp_arr[i - 1]);
        cv = cv_arr[i - 1] + fraction * (cv_arr[i] - cv_arr[i - 1]);
      }
      double const gamma = cp / cv;
      cp = 1000 * cp;
      return {cp, gamma};
    }

    double DAWallTempEstimation::NewtonRaphson() {
      double xs = startingGuess;
      double x0;
      double x1;
      double f0;
      double f1;
      double df;
      double fx;
      for (int i=0; i < maxIterations; i++) {
        x0 = xs - dx;
        x1 = xs + dx;
        f0 = HeatBalance(x0);
        f1 = HeatBalance(x1);
        df = f1 - f0;
        df = 0.5 * df / dx;
        fx = HeatBalance(xs);
        if (abs(fx) < tol_fx) {
          break;
        }
        if (abs(df) < tol_df) {
          dx = 10 * dx;
        } else {
          xs = xs - fx / df;
        }
        if (xs < xMin) {
          xs = 10;
          break;
        }
        if (xs > xMax){
          xs = 1000;
          break;
        }
      }
      return xs;
    }
}