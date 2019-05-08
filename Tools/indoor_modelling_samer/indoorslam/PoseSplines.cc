/*
                    Copyright 2013 University of Twente
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/

#include "BSplineFit.h"
#include "Rotation3D.h"
#include "Vector3D.h"

// Fit six splines to a sequence of pose parameters for a specified period of time
void FitPoseSplines(int order, double start_time, double end_time, 
                    int first_interval, int num_intervals,
                    int num_intervals_per_bspline, bool erase, BSplineFit &omega_spline,
                    BSplineFit &phi_spline, BSplineFit &kappa_spline,
					BSplineFit &X_spline, BSplineFit &Y_spline,
				    BSplineFit &Z_spline, const Rotation3D *rotations, 
					const Vector3D *translations)
{
  double interval_duration, time, previous_omega, previous_phi, previous_kappa,
         pi = 4.0 * atan(1.0), omega, phi, kappa;
  int    interval;
  
  interval_duration = (end_time - start_time) / num_intervals;
  omega_spline.Initialise(order, start_time, end_time,
                          num_intervals_per_bspline * interval_duration, erase);
  phi_spline.Initialise(order, start_time, end_time, 
                        num_intervals_per_bspline * interval_duration, erase);
  kappa_spline.Initialise(order, start_time, end_time, 
                          num_intervals_per_bspline * interval_duration, erase);
  X_spline.Initialise(order, start_time, end_time,
                      num_intervals_per_bspline * interval_duration, erase);
  Y_spline.Initialise(order, start_time, end_time,
                      num_intervals_per_bspline * interval_duration, erase);
  Z_spline.Initialise(order, start_time, end_time,
                      num_intervals_per_bspline * interval_duration, erase);
  for (interval=0, time=start_time + interval_duration / 2.0; 
       interval<num_intervals; interval++, time+=interval_duration) {
    rotations[first_interval+interval].DeriveAngles(omega, phi, kappa);
    if (interval > 0) { // Check on 2 pi jumps
      while (omega - previous_omega > pi) omega -= 2.0 * pi;
      while (previous_omega - omega > pi) omega += 2.0 * pi;
      while (phi - previous_phi > pi) phi -= 2.0 * pi;
      while (previous_phi - phi > pi) phi += 2.0 * pi;
      while (kappa - previous_kappa > pi) kappa -= 2.0 * pi;
      while (previous_kappa - kappa > pi) kappa += 2.0 * pi;
    }
    previous_omega = omega; previous_phi = phi; previous_kappa = kappa;
  	omega_spline.AddObservation(time, omega);
  	phi_spline.AddObservation(time, phi);
  	kappa_spline.AddObservation(time, kappa);
  	X_spline.AddObservation(time, translations[first_interval+interval].X());
  	Y_spline.AddObservation(time, translations[first_interval+interval].Y());
  	Z_spline.AddObservation(time, translations[first_interval+interval].Z());
  }
  printf("Fitting splines with %d knots\n",
         num_intervals / num_intervals_per_bspline + order - 1);
  omega_spline.FitSpline(); phi_spline.FitSpline(); kappa_spline.FitSpline();
  X_spline.FitSpline(); Y_spline.FitSpline(); Z_spline.FitSpline();
}

void ExtractScanLinePoses(int first_interval, double time_first_interval,
                          int num_intervals, double interval_duration,
                          const BSplineFit &omega_spline,
                          const BSplineFit &phi_spline,
						  const BSplineFit &kappa_spline,
					      const BSplineFit &X_spline, 
						  const BSplineFit &Y_spline,
				          const BSplineFit &Z_spline, 
						  Rotation3D *rotations, Vector3D *translations)
{
  double time;
  int    i, interval;
  
  for (i=0, interval=first_interval, time=time_first_interval; i<num_intervals;
       i++, interval++, time+=interval_duration) {
    rotations[interval] = Rotation3D(omega_spline.Value(time),
	                                 phi_spline.Value(time),
	                                 kappa_spline.Value(time));
	translations[interval] = Vector3D(X_spline.Value(time), Y_spline.Value(time),
	                                  Z_spline.Value(time));
  }
}
