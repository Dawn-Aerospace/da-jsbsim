//
// Created by caleb on 23/01/23.
//

#ifndef JSB_STATE_DAWALLTEMPESTIMATION_H
#define JSB_STATE_DAWALLTEMPESTIMATION_H

#include "FGFDMExec.h"
#include "FGAtmosphere.h"

namespace JSBSim {



class DAWallTempEstimation {
public:
    explicit DAWallTempEstimation(JSBSim::FGFDMExec *_fdmex);
    double GetWallTempEstimateCelsius(double chord);

private:
    FGFDMExec* fdmex_;
    std::shared_ptr<FGAtmosphere> atmosphere_;

    double HeatBalance(double estimate) const;
    std::tuple<double, double> static CalculateCpAndGamma(double estimate);
    double NewtonRaphson();

    double machSpeed;
    double chord_m;
    double airTemp_K;
    double airPressure_Pa;
    double kinematicViscosity;
    double absoluteViscosity;
    double soundSpeed_ms;
    double velocity_ms;
    double reynoldsNumber;
    double cfi, NN, C, N, a, b;

    double startingGuess=500;
    double tol=1E-2;
    double dx=1E-1;
    double maxIterations=30;
    double w = 0.76;
    double sg = 5.67E-8; //stefan-boltzmann constant
    double emissivity_ = 0.9; //emissivity anodised aluminum or white paint or black paint
    int flowType_ = 1; //flow type: 0 = laminar, 1 = turbulent

    double KELVIN_TO_CELSIUS_ = -273.15;
    double FEET_TO_METER_ = 0.3048;
    double RANKINE_TO_KELVIN_ = 1/1.8;
    double PSF_TO_PASCAL_ = 47.880208;
};
}
#endif //JSB_STATE_DAWALLTEMPESTIMATION_H
