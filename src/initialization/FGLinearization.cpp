/*
 * FGLinearization.cpp
 * Copyright (C) James Goppert 2011 <james.goppert@gmail.com>
 *
 * FGLinearization.h is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * FGLinearization.h is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FGInitialCondition.h"
#include "FGLinearization.h"

namespace JSBSim {

FGLinearization::FGLinearization(FGFDMExec * FDMExec, double h)
{
    fdm = FDMExec;
    //std::cout << "\nlinearization: " << std::endl;
    //std::cout << "sampling time: " << h <<std::endl;
    std::clock_t time_start=clock(), time_linDone;
    FGStateSpace ss(fdm);

    ss.x.add(new FGStateSpace::Latitude);
    ss.x.add(new FGStateSpace::Longitude);
    ss.x.add(new FGStateSpace::Alt);
    ss.x.add(new FGStateSpace::Vn);
    ss.x.add(new FGStateSpace::Ve);
    ss.x.add(new FGStateSpace::Vd);
    ss.x.add(new FGStateSpace::Vt);
    ss.x.add(new FGStateSpace::Alpha);
    ss.x.add(new FGStateSpace::Theta);
    ss.x.add(new FGStateSpace::Q);
    ss.x.add(new FGStateSpace::Gamma);

    
    // get propulsion pointer to determine type/ etc.
    auto engine0 = fdm->GetPropulsion()->GetEngine(0);
    FGThruster * thruster0 = engine0->GetThruster();

    if (thruster0->GetType()==FGThruster::ttPropeller)
    {
        ss.x.add(new FGStateSpace::Rpm0);
        // TODO add variable prop pitch property
        // if (variablePropPitch) ss.x.add(new FGStateSpace::PropPitch);
        int numEngines = fdm->GetPropulsion()->GetNumEngines();
        if (numEngines>1) ss.x.add(new FGStateSpace::Rpm1);
        if (numEngines>2) ss.x.add(new FGStateSpace::Rpm2);
        if (numEngines>3) ss.x.add(new FGStateSpace::Rpm3);
        if (numEngines>4) {
            std::cerr << "more than 4 engines not currently handled" << std::endl;
        }
    }

    ss.x.add(new FGStateSpace::Beta);
    ss.x.add(new FGStateSpace::Phi);
    ss.x.add(new FGStateSpace::P);
    ss.x.add(new FGStateSpace::R);
    ss.x.add(new FGStateSpace::Psi);


    ss.u.add(new FGStateSpace::ThrottleCmd);
    ss.u.add(new FGStateSpace::DeCmd);
    ss.u.add(new FGStateSpace::DaCmd);
    ss.u.add(new FGStateSpace::DrCmd);

    // state feedback
    ss.y = ss.x;

    x0 = ss.x.get();
    u0 = ss.u.get();
    y0 = x0; // state feedback

    double dt_saved = fdm->GetDeltaT();
    fdm->SuspendIntegration();
    ss.linearize(x0, u0, y0, A, B, C, D, h);
    fdm->ResumeIntegration();
    fdm->Setdt(dt_saved);

    x_names = ss.x.getName();
    u_names = ss.u.getName();
    y_names = ss.y.getName();
    x_units = ss.x.getUnit();
    u_units = ss.u.getUnit();
    y_units = ss.y.getUnit();



 time_linDone = std::clock();
    std::cout << "  Linearization computation time: " << (time_linDone - time_start)/double(CLOCKS_PER_SEC) << " s\n" << std::endl;

/*
int width=10;
    std::cout.precision(10);
    std::cout
        << std::fixed
        << std::right
        << "\nA=\n" << std::setw(width) << A
        << "\nB=\n" << std::setw(width) << B
        << "\nC=\n" << std::setw(width) << C
        << "\n* note: C should be identity, if not, indicates problem with model"
        << "\nD=\n" << std::setw(width) << D
        << std::endl;
*/
int Digits =18;
  ofstream myfile;
  myfile.open ("Linearization_Output_LTI.m");
  myfile << "A="<< setprecision(Digits) << A<< ";";
  myfile << endl << endl;
  myfile << "B="<< setprecision(Digits) << B<< ";";
  myfile << endl << endl;
    myfile << "C="<< setprecision(Digits) << C<< ";";
  myfile << endl << endl;
  myfile << "D="<< setprecision(Digits) << D<< ";";
  myfile << endl << endl;
  myfile << "x0="<< setprecision(Digits) << x0<< ";";
  myfile << endl << endl;
  myfile << "u0="<<setprecision(Digits) << u0<< ";";
  myfile << endl << endl;
  myfile << "y0="<< setprecision(Digits) << y0<< ";";
  myfile << endl << endl;

  myfile << "state_names= {" ;
  for (unsigned int i=0;i<x_names.size();i++) {
  myfile <<"'"<<x_names[i]<<"' ";
  }
  myfile << "};" << endl;

  myfile << "state_units= {" ;
  for (unsigned int i=0;i<x_units.size();i++) {
  myfile <<"'"<<x_units[i]<<"' ";
  }
  myfile << "};" << endl;

  myfile << "input_names= {" ;
  for (unsigned int i=0;i<u_names.size();i++) {
  myfile <<"'"<<u_names[i]<<"' ";
  }
  myfile << "};" << endl;

  myfile << "input_units= {" ;
  for (unsigned int i=0;i<u_units.size();i++) {
  myfile <<"'"<<u_units[i]<<"' ";
  }
  myfile << "};" << endl;

    myfile << "output_names= {" ;
  for (unsigned int i=0;i<y_names.size();i++) {
  myfile <<"'"<<y_names[i]<<"' ";
  }
  myfile << "};" << endl;

  myfile << "output_units= {" ;
  for (unsigned int i=0;i<y_units.size();i++) {
  myfile <<"'"<<y_units[i]<<"' ";
  }
  myfile << "};" << endl;


  myfile.close();
}

} // JSBSim

// vim:ts=4:sw=4
