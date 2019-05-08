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

/* Class for fitting B-spline curves to data. Knots are uniformly distributed.

   Example code:
   	
   #include "BSplineFit.h"
   
   {
   	  BSplineFit spline_curve;
   	  
   	  // Initialise spline of order "order" on range t_start-t_end with a
   	  // given interval size between the knots.
   	  spline_curve.Initialise(order, t_start, t_end, interval_size);
   	  
   	  // Add data
   	  spline_curve.AddObservation(t1, value1);
   	  spline_curve.AddObservation(t2, value2);
   	  ...
   	  
   	  // Fit spline
   	  spline_curve.FitSpline();
   	  
   	  // Retrieve spline value at a certain location
   	  value = spline_curve.Value(t);
   	  
   }
*/
   	  
#include <stdlib.h>
#include <vector>
#include "NormalEquations.h"

class BSplineFit
{

protected:
	/// Uniform or not
	bool uniform;
	
    /// Spline order
    int order;

    /// Spline curve start
    double curve_start;
    
    /// Spline curve end
    double curve_end;

    /// Number of internal knots
    int num_knots;
    
    /// Knot interval size (for uniform splines)
    double knot_interval_size;
    
    /// Knot vector (for non-uniform splines)
    std::vector<double> knots;
    
    /// Number of spline functions
    int num_splines;
    
    /// Spline coefficients
    double *spline_coefficients;
    
    /// Row of design matrix
    double *a_row;
    
    /// Number of observations
    int num_obs;
    
    /// Normal equation system
    NormalEquations neq;

private:
	
    /// Return the location of a knot
    /** Knot 0 is the first knot of the interval on which the
        first spline is defined. Knot 1 is the first knot of the
        interval on which the second spline is defined. It coincides
        with the second knot on the interval on which the first spline
        is defined. Etc.
	    @param index of the knot
	    @return knot location
	*/
	double Knot(int index) const;
	
	/// Return the value of a B-spline
	/** Recursive function to determine a B-spline value
	    @param location Location within spline
	    @param interval Spline interval index
	    @param recursion_order Spline order in recursive formula
	    @return B-spline value
	*/
	double BSplineValue(double location, int interval,
	                    int recursion_order) const;

	/// Return the value of a B-spline derivative
	/** Recursive function to determine the value of a B-spline derivative
	    @param location Location within spline with unit knot distance
	    @param interval Spline interval
	    @param recursion_order Spline order in recursive formula
	    @param derivative Derivative number (first, second, ...)
	    @return B-spline derivative value
	*/
    double BSplineDerivative(double location, int interval,
                             int recursion_order, int derivative) const;

public:
    /// Copy constructor
    BSplineFit(const BSplineFit &spline)
      { spline_coefficients = a_row = NULL; neq.Initialise(1);
	    *this = spline; }
      			
    /// Copy assignment
	BSplineFit& operator = (const BSplineFit &spline);
	
	/// Index of first B-spline used at a position
    int FirstBSplineIndex(double location) const;
    
    /// B-spline at position in curve for a given spline index
    double BSplineValue(double location, int index) const;

    /// B-spline derivative at position in curve for a given spline index
    double BSplineDerivative(double location, int index, int derivative) const;

    /// B-spline at position in curve for a given spline start location
//    double BSplineValue(double location, double bspline_start_location) const;
    
    /// Default constructor
	BSplineFit() { num_knots = 0; spline_coefficients = a_row = NULL;
	               uniform = true; neq.Initialise(1); }
		
    /// Define range, order and knot interval for a uniform spline
    /** Construct spline range, order and knot interval from
        @param order Spline order
        @param start Start of the spline curve
        @param end   End of the spline curve
        @param numk  Number of internal knots
    */
	BSplineFit(int order, double start, double end, int numk)
	  { Initialise(order, start, end, numk, false); }	

    /// Initialise range, order and knot interval for a uniform spline
    /** Initialise spline range, order and knot interval from
        @param order Spline order
        @param start Start of the spline curve
        @param end   End of the spline curve
        @param numk  Number of internal knots
        @param erase Free previously allocated memory
    */
    void Initialise(int order, double start, double end, int numk,
	                bool erase=false);

    /// Initialise range, order and knot interval for a uniform spline
    /** Initialise spline range, order and knot interval from
        @param order Spline order
        @param start Start of the spline curve
        @param end   End of the spline curve
        @param interval  Knot interval
        @param erase Free previously allocated memory
    */
    void Initialise(int order, double start, double end, double interval,
	                bool erase=false);

    /// Initialise a non-uniform spline
    /** Initialise a non-uniform spline
        @param order Spline order
		@param knot_locations Vector of knot locations
        @param erase Free previously allocated memory
    */
    void Initialise(int order, std::vector<double> knot_locations,
	                bool erase=false);
	                
    /// Erase spline coefficients
    void Erase();
    
    /// Add observation for spline fitting
    /** @param t Location of spline value
        @param v Observed spline value at t
        @param weight Weight of the observation
        @param print Print coefficients (row of design matrix)
        @return false if t outside spline curve range
    */
    bool AddObservation(double t, double v, double weight=1.0,
	                    bool print=false);
    
    /// Add derivative constraint for spline fitting
    /** @param t Location of spline derivative
        @param v Desired derivative value at t
        @param derivative Derivative number (first, second, ...)
        @param weight Weight of the constraint
        @param print Print coefficients (row of design matrix)
        @return false if t outside spline curve range
    */
    bool AddConstraint(double t, double v, int derivative, double weight=1.0,
	                   bool print=false);

    /// Fit a spline curve to the observations
    /** @return Error codes: 0 - success
	                         1 - insufficient observations
                             2 - singular equation system
    */
    int FitSpline();
    
    /// B-Spline value at specified location
    double Value(double t) const;
    
    /// B-Spline derivative at specified location
    double Derivative(double t, int derivative) const;
    
    /// Return order
    int Order() const {return order;}    
    
    /// Return the number of splines
    int NumberOfSplines() const {return num_splines;}
    
    /// Return the number of knots
    int NumberOfKnots() const {return num_knots;}
    
    /// Return the curve start
    double Start() const {return curve_start;}
    
    /// Return the curve end
    double End() const {return curve_end;}
	               
	/// Return the knot interval
	double KnotInterval() const {return knot_interval_size;}
    
    /// Return the spline coefficients
    double *SplineCoefficients() {return spline_coefficients;}

    /// Extrapolate spline to a new range
    int Extrapolate(double new_start, double new_end);

    /// Set up a linear spline with two knots
    void SetLinear(double start, double end,
	               double start_value, double end_value);
};
