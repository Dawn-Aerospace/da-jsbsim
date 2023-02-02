//
// Created by caleb on 23/01/23.
//

#ifndef JSB_STATE_DAWALLTEMPESTIMATION_H
#define JSB_STATE_DAWALLTEMPESTIMATION_H

#include "FGFDMExec.h"
#include "FGAtmosphere.h"
#include "FGJSBBase.h"

namespace JSBSim
{
    class DAWallTempEstimation : public FGJSBBase
    {
    public:
        DAWallTempEstimation(){};
        DAWallTempEstimation(JSBSim::FGFDMExec *_fdmex);
        double GetWallTempEstimateCelsius(double chord, double _machSpeed);

    protected:
        std::shared_ptr<FGAtmosphere> atmosphere_;
        double machSpeed;
        double airTemp_K;
        double airPressure_Pa;
        double kinematicViscosity;
        double absoluteViscosity;
        double soundSpeed_ms;
        double maxIterations = 30;
        int flowType_ = 1; // flow type: 0 = laminar, 1 = turbulent
        virtual void SetInputs();

        double HeatBalance(double estimate) const;
        std::tuple<double, double> static CalculateCpAndGamma(double temperature);
        double NewtonRaphson(double dx);

    private:
        FGFDMExec *fdmex_;
        double velocity_ms;
        double cfi, NN, a, b;

        double startingGuess = 100;
        double tol_fx = 1E-2;
        double tol_df = 1E-16;
        int xMin = 10;
        int xMax = 10000;
        double w = 0.76;
        double sg = 5.67E-8;      // stefan-boltzmann constant
        double emissivity_ = 0.9; // emissivity anodised aluminum or white paint or black paint
    };
}
#endif // JSB_STATE_DAWALLTEMPESTIMATION_H
