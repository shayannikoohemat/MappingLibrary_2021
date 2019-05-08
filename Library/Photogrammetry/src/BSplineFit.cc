/*
                     Copyright 2014 University of Twente
 
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

#include <stdio.h>
#include <string.h>
#include "BSplineFit.h"
#include "normal_equations.h"

////////////////////////////////////////////////////////////////////////////////
//                          Private functions
//


// Knot location, used for both uniform and non-uniform splines

double BSplineFit::Knot(int index) const
{
  // Check index validity
  if (index < 0 || index >= num_knots + 2 * (order - 1)) {
  	printf("Error: Invalid index %d to BSplineFit::Knot for a curve with\n", index);
  	printf("       %d internal and %d external knots.\n", num_knots, 2 * (order - 1));
  	exit(0);
  }
  
  // Calculate knot location in case of uniform splines
  if (uniform)
    return curve_start + (double) (index - order + 1) * knot_interval_size;
    
  // Take knot location from knot vector for non-uniform splines
  return knots[index];
}


// Recursive definition of a B-spline value

double BSplineFit::BSplineValue(double location, int interval,
                                int recursion_order) const
{
  if (recursion_order == 1) {
//  	if (interval < 0 || interval >= knots.size()) return 0.0;
  	if (interval == num_knots + 2 * (order - 1) - 1) { // Special case for end point
	  if (location == Knot(interval)) return 1.0;
	  else return 0.0;
  	}
    if (location >= Knot(interval) && location < Knot(interval+1)) 
      return 1.0;
    else return 0.0;
  }

  // Recursive definition for higher orders
  return BSplineValue(location, interval, recursion_order - 1) *
         (location - Knot(interval)) /
		 (Knot(interval+recursion_order-1) - Knot(interval)) +
		 BSplineValue(location, interval+1, recursion_order - 1) *
		 (Knot(interval+recursion_order) - location) /
		 (Knot(interval+recursion_order) - Knot(interval+1));
}


// Recursive definition of a B-spline derivative

double BSplineFit::BSplineDerivative(double location, int interval,
                                     int recursion_order, int derivative) const
{
  // The first derivative is a linear function of the B-spline
  // values of the next lower order.
  if (derivative == 1) {
  	return ((double) recursion_order - 1.0) *
  	       (BSplineValue(location, interval, recursion_order-1) /
  	        (Knot(interval+recursion_order-1) - Knot(interval)) -
  	        BSplineValue(location, interval+1, recursion_order-1) /
  	        (Knot(interval+recursion_order) - Knot(interval+1)));
  }
  
  // Recursively derive higher order derivatives  
  return ((double) recursion_order - 1.0) *
         (BSplineDerivative(location, interval, recursion_order-1,
		                    derivative-1) /
		  (Knot(interval+recursion_order-1) - Knot(interval)) -
		  BSplineDerivative(location, interval+1, recursion_order-1,
		                    derivative-1) /
		  (Knot(interval+recursion_order) - Knot(interval+1))); 
}


////////////////////////////////////////////////////////////////////////////////
//                          Public functions
//


// Copy Assignment

BSplineFit & BSplineFit::operator=(const BSplineFit &spline)
{
  // Copy spline parameters by re-initialisation
  if (uniform)
    Initialise(spline.order, spline.curve_start, spline.curve_end,
               spline.knot_interval_size, true);
  else
    Initialise(spline.order, spline.knots);

  // Copy spline data
  num_obs = spline.num_obs;
  memcpy((void *) spline_coefficients, (void *) spline.spline_coefficients,
         num_splines * sizeof(double));
  memcpy((void *) a_row, (void *) spline.a_row, num_splines * sizeof(double));
  neq = spline.neq;
         
  return *this;
}

// Spline initialisation for uniform splines

void BSplineFit::Initialise(int spline_order, double start, double end, int numk,
                            bool erase)
{
  if (numk < 2) {
  	printf("Error: minimum number of knots is 2\n");
  	return;
  }
  knot_interval_size = (end - start) / (numk - 1);
  Initialise(spline_order, start, end, knot_interval_size, erase);
}


// Spline initialisation for uniform splines

void BSplineFit::Initialise(int spline_order, double start, double end,
                            double interval, bool erase)
{
  uniform             = true;
  order               = spline_order;
  curve_start         = start;
  curve_end           = end;
  knot_interval_size  = interval;
  num_knots           = (int) ((end - start - 0.0001) / interval) + 2;
  num_splines         = num_knots + order - 2;
  num_obs             = 0;
  if (erase) Erase();
  spline_coefficients = (double *) malloc(num_splines * sizeof(double));
  a_row               = (double *) malloc(num_splines * sizeof(double));
  if (!spline_coefficients || !a_row) {
  	printf("Error allocating memory in BSplineFit::Initialise(...)\n");
  	exit(0);
  }
  if (neq.Initialise(num_splines, num_splines, order, true) != 0) {
  	printf("Error initialising normal equations in BSplineFit::Initialise(...)\n");
  	exit(0);
  }
}

// Spline initialisation for non-uniform splines

void BSplineFit::Initialise(int spline_order,
                            std::vector<double> internal_knots, bool erase)
{
  int i;
  bool debug=false;
  
  if (erase) Erase();
  uniform             = false;
  order               = spline_order;
  num_knots           = internal_knots.size();
  curve_start         = internal_knots[0];
  curve_end           = internal_knots[num_knots-1];
  knot_interval_size  = 0.0;
  num_splines         = num_knots + order - 2;
  num_obs             = 0;
  spline_coefficients = (double *) malloc(num_splines * sizeof(double));
  a_row               = (double *) malloc(num_splines * sizeof(double));
  if (!spline_coefficients || !a_row) {
  	printf("Error allocating memory in BSplineFit::Initialise(...)\n");
  	exit(0);
  }
  if (neq.Initialise(num_splines, num_splines, order, true) != 0) {
  	printf("Error initialising normal equations in BSplineFit::Initialise(...)\n");
  	exit(0);
  }
  // Initialise knots vector
  // Copy internal knots
  knots.insert(knots.begin(), internal_knots.begin(), internal_knots.end());
  // Add (order - 1) external knots at the start with the same spacing
  // as the spacing between the first two internal knots
  for (i=0; i<order-1; i++)
    knots.insert(knots.begin(), 2.0*knots[0] - knots[1]);
  // Add (order - 1) external knots at the end with the same spacing
  // as the spacing between the last two internal knots
  for (i=0; i<order-1; i++)
    knots.push_back(2.0 * (*(knots.end()-1)) - *(knots.end()-2));
  if (debug) {
    printf("Initialised knots:");
    for (i=0; i<num_knots+2*order-2; i++) printf("  %.2f", knots[i]);
    printf("\n");
  }
}


// Erase allocated arrays

void BSplineFit::Erase()
{
  if (spline_coefficients) {free(spline_coefficients); spline_coefficients=NULL;}
  if (a_row) {free(a_row); a_row=NULL;}
  if (!uniform && !knots.empty()) knots.erase(knots.begin(), knots.end());
  neq.FreeMemory();
}

// Index of first B-spline used at a position

int BSplineFit::FirstBSplineIndex(double location) const
{
  int spline_index, lo, hi, mid;
  
  if (uniform) {
    spline_index = (int) ((location - curve_start) / knot_interval_size);
    if (spline_index < 0) spline_index = 0;
    return spline_index;
  }
  
  // Non-uniform
  else { // Search for surrounding knots
    if (location < knots[order-1]) return 0;
    if (location > knots[num_knots-1+order-1]) return num_knots - 1;
    lo = order - 1;
    hi = num_knots - 1 + order - 1;
    while (lo <= hi) {
      mid = (hi + lo) / 2;
      if (location < knots[mid]) {
        hi = mid - 1;
      }
	  else if (location > knots[mid]) {
        lo = mid + 1;
      }
	  else {
        return mid - order + 1;
      }
    }
    // lo == hi + 1
    return hi - order + 1;
  }
}
    

double BSplineFit::BSplineValue(double location, int index) const
{
  // Check index validity
  if (index < 0 || index >= num_knots + 2 * (order - 1) - 1) {
  	printf("Error: Invalid index %d to BSplineFit::BSplineValue for a curve with\n", index);
  	printf("       %d internal and %d external knots.\n", num_knots, 2 * (order - 1));
  	exit(0);
  }
  
  // Use the private recursive function for B-spline value calculation
  return BSplineValue(location, index, order);
}

double BSplineFit::BSplineDerivative(double location, int index, int derivative) const
{
  if (derivative >= order) return 0.0;
  return BSplineDerivative(location, index, order, derivative);	
}

// B-spline at position in curve for a given spline start location
/* Obsolete ??
double BSplineFit::BSplineValue(double location, double bspline_start_location) const
{
  double unit_location;

  unit_location = (location - curve_start - bspline_start_location) / knot_interval_size;
  if (unit_location < 0.0 || unit_location >= (double) order) return 0.0;
  return UnitBSplineValue(unit_location, 0, order);
}
*/

bool BSplineFit::AddObservation(double t, double v, double weight, bool print)
{
  int    j, spline_index, first_spline_index;
  
  // Initialise all coefficients
  memset((void *) a_row, 0, num_splines * sizeof(double));
  
  // Set the coefficients of these splines
  first_spline_index = FirstBSplineIndex(t);
  for (j=0, spline_index=first_spline_index; j<order; j++, spline_index++) {
  	if (spline_index < num_splines)
  	  a_row[spline_index] = BSplineValue(t, spline_index);
  }
  num_obs++;
  
  if (print) {
  	for (j=0; j<num_splines; j++) printf("%7.4f ", a_row[j]);
  	printf("\n");
  }

  // Update the normal equation system
  neq.AddObservation(a_row, v, weight, first_spline_index, 
                     first_spline_index+order-1);
  
  return true;
}

bool BSplineFit::AddConstraint(double t, double v, int derivative,
                               double weight, bool print)
{
  int j, spline_index, first_spline_index;
  
  // Initialise all coefficients
  memset((void *) a_row, 0, num_splines * sizeof(double));
  
  // Set the coefficients of these splines
  first_spline_index = FirstBSplineIndex(t);
  for (j=0, spline_index=first_spline_index; j<order; j++, spline_index++) {
  	if (spline_index < num_splines)
  	  a_row[spline_index] = BSplineDerivative(t, spline_index, derivative);
  }
  num_obs++;
  
  if (print) {
  	for (j=0; j<num_splines; j++) printf("%7.4f ", a_row[j]);
  	printf("\n");
  }
    
  // Update the normal equation system
  neq.AddObservation(a_row, v, weight, first_spline_index, 
                     first_spline_index+order-1);
}

// Estimate spline coefficients

int BSplineFit::FitSpline()
{
  double rcond, *solution;
  int i, j;
  
  if (num_obs < num_splines) return 1; // Insufficient observations
  
  // Solve equation system. Use Cholesky for band matrices.
  solution = neq.Solve();
  
  // Transfer coefficients
  for (int i=0; i<num_splines; i++) spline_coefficients[i] = solution[i];
  
  return 0;
}


// B-Spline value at specified location

double BSplineFit::Value(double t) const
{
  int    j, spline_index;
  double value=0.0;
  
  for (j=0, spline_index=FirstBSplineIndex(t); j<order; j++, spline_index++) {
  	if (spline_index < num_splines)
      value += spline_coefficients[spline_index] * BSplineValue(t, spline_index);
  }
  
  return value;
}

// B-Spline derivative value at specified location

double BSplineFit::Derivative(double t, int derivative) const
{
  int    j, spline_index;
  double value=0.0;
  
  for (j=0, spline_index=FirstBSplineIndex(t); j<order; j++, spline_index++) {
  	if (spline_index < num_splines)
      value += spline_coefficients[spline_index] * 
	           BSplineDerivative(t, spline_index, derivative);
  }
  
  return value;
}

// Extrapolate spline to a new range
int BSplineFit::Extrapolate(double new_start, double new_end)
{
  double value0, value1;
  
  if (order > 2) return 1; // Only implemented for constant linear splines
  if (num_knots > 2) return 2; // Only implemented for at most two knots
  if (spline_coefficients == NULL) return 5; // No spline defined yet
  
  // Determine new spline coefficients
  if (order == 1) {
    curve_start = new_start;
    curve_end   = new_end;
  }
  else if (order == 2) {
  	if (num_knots == 2) {
  	  SetLinear(new_start, new_end,
  	            spline_coefficients[0] + (new_start - curve_start) *
  	            (spline_coefficients[1] - spline_coefficients[0]) /
			    (curve_end - curve_start),
	            spline_coefficients[0] + (new_end - curve_start) *
  	            (spline_coefficients[1] - spline_coefficients[0]) /
			    (curve_end - curve_start));
  	}
  	else {
  	  printf("Extrapolate not implemented for %d knots\n", num_knots);
  	  return 3;
  	}
  }
  else {
  	printf("Invalid spline order %d\n", order);
  	return 4;
  }
  
  return 0;
}

void BSplineFit::SetLinear(double start, double end,
                           double value_start, double value_end)
{
  Initialise(2, start, end, 2, (spline_coefficients!=NULL));
  spline_coefficients[0] = value_start;
  spline_coefficients[1] = value_end;
}

