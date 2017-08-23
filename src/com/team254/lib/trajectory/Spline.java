package com.team254.lib.trajectory;

import com.team254.lib.util.ChezyMath;

/**
 * 
 * This class defines methods for doing cubic and quintic spline interpolation between 2 points.
 * 
 * It encapsulates spline data by defnining a static method for interpolating the spline 
 * and instance methods for getting point data for splines
 * 
 * It defines the functions for:
 * 	-Getting the coefficients for the spline
 * 	-Getting the slope, position, or second derivative at any percentage of the way through the spline 
 * 		(Note that this is just an input value for the spline function mapped to [0, 1],
 * 		 not an actual arc-length parameterization)
 * 
 * Note that the spline interpolation automatically sets the second derivatives at each point to 0 for 
 * 	quintic splines (for stability reasons), so the arguments are always the values sfor the first 
 * 	point, the second point, and the slope angles for each.
 * 
 * 
 * Do cubic spline interpolation between points.
 *
 * @author Art Kalb
 * @author Jared341
 */
public class Spline {

  //this class encapsulates the type of spline to be interpolated.
  //TODO: replace this idiotic class with an enum
  public static class Type {

    private final String value_;

    private Type(String value) {
      this.value_ = value;
    }

    public String toString() {
      return value_;
    }
  }

  // Cubic spline where positions and first derivatives (angle) constraints will
  // be met but second derivatives may be discontinuous.
  public static final Type CubicHermite = new Type("CubicHermite");

  // Quintic spline where positions and first derivatives (angle) constraints
  // will be met, and all second derivatives at knots = 0.
  public static final Type QuinticHermite = new Type("QuinticHermite");

  
  //instance variable for spline coefficients
  Type type_; //note that type_ is never used TODO: fix
  double a_;  // ax^5
  double b_;  // + bx^4
  double c_;  // + cx^3
  double d_;  // + dx^2
  double e_;  // + ex
  // f is always 0 for the spline formulation we support.

  // The offset from the world frame to the spline frame.
  // Add these to the output of the spline to obtain world coordinates.
  double y_offset_;
  double x_offset_;
  double knot_distance_;
  double theta_offset_;
  double arc_length_;

  Spline() {
    // All splines should be made via the static interface <--not sure why the this is the case
    arc_length_ = -1;
  }

  /**
   * Compares 2 input values and returns true if they are approximately equal
   * (used for checking slope edge cases in reticuation e.g if a vertical slope exists)
   */
  private static boolean almostEqual(double x, double y) {
    return Math.abs(x - y) < 1E-6;
  }

  public static boolean reticulateSplines(WaypointSequence.Waypoint start,
          WaypointSequence.Waypoint goal, Spline result, Type type) {
    return reticulateSplines(start.x, start.y, start.theta, goal.x, goal.y,
            goal.theta, result, type);
  }

  /**
   * Generates a spline based on given spline type.
   * 
   * The hermetic spline generated is rotated so the first point is on the 
   * origin, and the x-axis goes through both the forst point and the second point. 
   * The domain of the function between the 2 points is on [0, (distance between the 2 points)]
   * 
   * @param x0 The x commponent of the starting position
   * @param y0 The y componenet of the starting position
   * @param theta0 The starting rotation of the starting position
   * @parm x1 The x component of the ending position 
   * @param y1 The y component of the ending position
   * @param result The spline to output values to
   * @param type The type of spline to reticulate
   * @returns Whether the spline reticulation succeeded
   */
  public static boolean reticulateSplines(double x0, double y0, double theta0,
          double x1, double y1, double theta1, Spline result, Type type) {
    System.out.println("Reticulating splines...");
    //assign type to spline
    result.type_ = type;

    // Transform x to the origin
    result.x_offset_ = x0;
    result.y_offset_ = y0;
    
    //calculate the distance between the 2 points (not sure why this is called a hat when it's a magnitude)
    double x1_hat = Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    
    //cancel if both points are the same
    if (x1_hat == 0) {
      return false;
    }
    
    //set direct point to point distance
    result.knot_distance_ = x1_hat;
    
    //calculate angle between the 2 points
    result.theta_offset_ = Math.atan2(y1 - y0, x1 - x0);
    
    //calculate point angles relative to theta_offset_ and transform onto [-pi, pi]
    //these angles are now used for the new coordinate system
    double theta0_hat = ChezyMath.getDifferenceInAngleRadians(
            result.theta_offset_, theta0);
    double theta1_hat = ChezyMath.getDifferenceInAngleRadians(
            result.theta_offset_, theta1);
    
    
    // We cannot handle vertical slopes in our rotated, translated basis.
    // This would mean the user wants to end up 90 degrees off of the straight
    // line between p0 and p1.
    if (almostEqual(Math.abs(theta0_hat), Math.PI / 2)
            || almostEqual(Math.abs(theta1_hat), Math.PI / 2)) {
      return false;
    }
    
    // We also cannot handle the case that the end angle is facing towards the
    // start angle (total turn > 90 degrees).
    if (Math.abs(ChezyMath.getDifferenceInAngleRadians(theta0_hat,
            theta1_hat))
            >= Math.PI / 2) {
      return false;
    }
    
    // Turn angles into derivatives (slopes)
    double yp0_hat = Math.tan(theta0_hat);
    double yp1_hat = Math.tan(theta1_hat);

    //These Hermitic curve formulas will differ sligtly from those found in formal papers
    //eg: https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf
    //because:
    //1) Because both ponts are on the x axis, their y values are 0, simplifying the formulas
    //2) Because the functon's domain is on [0, x1_hat] and not [0,1], x1_hat works its way into the formulas
    
    //TODO: make a decent whitepaper explaining what's going on here
    
    if (type == CubicHermite) {
      // Calculate the cubic spline coefficients
      result.a_ = 0;
      result.b_ = 0;
      result.c_ = (yp1_hat + yp0_hat) / (x1_hat * x1_hat);
      result.d_ = -(2 * yp0_hat + yp1_hat) / x1_hat;
      result.e_ = yp0_hat;
    } else if (type == QuinticHermite) {
      result.a_ = -(3 * (yp0_hat + yp1_hat)) / (x1_hat * x1_hat * x1_hat * x1_hat);
      result.b_ = (8 * yp0_hat + 7 * yp1_hat) / (x1_hat * x1_hat * x1_hat);
      result.c_ = -(6 * yp0_hat + 4 * yp1_hat) / (x1_hat * x1_hat);
      result.d_ = 0;
      result.e_ = yp0_hat;
    }

    //splines calculated, operation succeeded.
    return true;
  }

  /**
   * Calculates the arc length of the spline
   */
  public double calculateLength() {
	//return value if already calculated
    if (arc_length_ >= 0) {
      return arc_length_;
    }
    
    //sample count for arc-length integral (1/dt)
    final int kNumSamples = 100000;
    
    //instantiate integral vars
    double arc_length = 0;
    double t, dydt;
    
    //get starting values for 0
    double integrand, last_integrand
            = Math.sqrt(1 + derivativeAt(0) * derivativeAt(0)) / kNumSamples;
    
    //integrate http://tutorial.math.lamar.edu/Classes/CalcII/ArcLength_files/eq0016P.gif
    //integrate from 0 to 1
    for (int i = 1; i <= kNumSamples; ++i) {
      t = ((double) i) / kNumSamples;
      dydt = derivativeAt(t);
      integrand = Math.sqrt(1 + dydt * dydt) / kNumSamples;
      arc_length += (integrand + last_integrand) / 2;
      last_integrand = integrand;
    }
    
    //scale arc length from [0,1] scale to [0, (point distance)]
    arc_length_ = knot_distance_ * arc_length;
    
    return arc_length_;
  }

  /**
   * Returns the "percentage", or percent of x1_hat for which t = percent * x1_hat
   * is the given distance along the spline
   */
  public double getPercentageForDistance(double distance) {
	//set sample count (1/dt)
    final int kNumSamples = 100000;
    
    //set starting variables for arc-length integral
    double arc_length = 0;
    double t = 0;
    double last_arc_length = 0;
    double dydt;
    
    //set starting integral values
    double integrand, last_integrand
            = Math.sqrt(1 + derivativeAt(0) * derivativeAt(0)) / kNumSamples;
    distance /= knot_distance_;
    
    //Integrate arc-length, checking if the current arc-length is >= the distance argument
    //thus the t value when the for loop breaks is the t value for the given distance
    for (int i = 1; i <= kNumSamples; ++i) {
      //get t value on [0, 1]
      t = ((double) i) / kNumSamples;
      
      dydt = derivativeAt(t);
      
      //integrate using formula
      integrand = Math.sqrt(1 + dydt * dydt) / kNumSamples;
      arc_length += (integrand + last_integrand) / 2;
      
      //check if distance passed
      if (arc_length > distance) {
        break;
      }
      
      //set prior vals
      last_integrand = integrand;
      last_arc_length = arc_length;
    }

    // Interpolate between samples.
    double interpolated = t;
    if (arc_length != last_arc_length) {
      interpolated += ((distance - last_arc_length)
              / (arc_length - last_arc_length) - 1) / (double) kNumSamples;
    }
    
    return interpolated;
  }

  /**
   * Returns the x and y values along the curve for a given t mapped onto [0, 1]
   * which is a percentage...sorta (it's a time percentage, not a distance one).
   */
  public double[] getXandY(double percentage) {
	  //instantiate array to return the values TODO: implement Vector2
    double[] result = new double[2];

    //force percentage arg onto [0, 1]
    percentage = Math.max(Math.min(percentage, 1), 0);
    
    //calculate relative x and y (again...why are these hats?)
    double x_hat = percentage * knot_distance_;
    double y_hat = (a_ * x_hat + b_) * x_hat * x_hat * x_hat * x_hat
            + c_ * x_hat * x_hat * x_hat + d_ * x_hat * x_hat + e_ * x_hat;

    //get x any components of unit theta offset
    double cos_theta = Math.cos(theta_offset_);
    double sin_theta = Math.sin(theta_offset_);

    //translate values back onto original basis
    //this is just an implelementation a the rotation matrix for theta_offset
    //https://en.wikipedia.org/wiki/Rotation_matrix
    result[0] = x_hat * cos_theta - y_hat * sin_theta + x_offset_;
    result[1] = x_hat * sin_theta + y_hat * cos_theta + y_offset_;
    
    //return result array
    return result;
  }

  /**
   * Returns the y vaule of the curve for a given t mapped onto [0,1]
   * 
   * TODO: just make this function call getXandY since it's 95% the same.
   */
  public double valueAt(double percentage) {
	//force percent arg onto [0,1]
    percentage = Math.max(Math.min(percentage, 1), 0);
    
    //calculate relative x and y 
    double x_hat = percentage * knot_distance_;
    double y_hat = (a_ * x_hat + b_) * x_hat * x_hat * x_hat * x_hat
            + c_ * x_hat * x_hat * x_hat + d_ * x_hat * x_hat + e_ * x_hat;

    //get x any components of unit theta offset
    double cos_theta = Math.cos(theta_offset_);
    double sin_theta = Math.sin(theta_offset_);

    //return y value of rotation matrix of theta_offset
    double value = x_hat * sin_theta + y_hat * cos_theta + y_offset_;
    return value;
  }

  /**
   * Returns the derivative of the spline at a given value of t scaled to [0, 1]
   * 
   * note that this derivative isnt rotated back into the original coords
   */
  private double derivativeAt(double percentage) {
	//force percentage arg onto [0, 1]
    percentage = Math.max(Math.min(percentage, 1), 0);

    //get x position of percentage by re-scaling to [0, (point distance)]
    double x_hat = percentage * knot_distance_;
    
    //get derivative at x_hat
    double yp_hat = (5 * a_ * x_hat + 4 * b_) * x_hat * x_hat * x_hat + 3 * c_ * x_hat * x_hat
            + 2 * d_ * x_hat + e_;

    return yp_hat;
  }

  /**
   * Reutrns the second deriveative of the spline at a given value of t scaled to [0, 1]
   * 
   */
  private double secondDerivativeAt(double percentage) {
	//force percentage arg onto [0, 1]
    percentage = Math.max(Math.min(percentage, 1), 0);

    
 	//get x position of percentage by re-scaling to [0, (point distance)]
    double x_hat = percentage * knot_distance_;
    
    //get second derivative at x_hat
    double ypp_hat = (20 * a_ * x_hat + 12 * b_) * x_hat * x_hat + 6 * c_ * x_hat + 2 * d_;

    return ypp_hat;
  }

  /**
   * Returns the angle of the derivative at a given t scaled to [0, 1]
   * 
   * note that this *is* rotated back to the original coordinate system
   */
  public double angleAt(double percentage) {
	//take atan of derivative + theta_offset (to rotate back to original coords) to get angle
    double angle = ChezyMath.boundAngle0to2PiRadians(
            Math.atan(derivativeAt(percentage)) + theta_offset_);
    
    return angle;
  }

  /**
   * Returns change in angle / change in time of the derivative for a given t scaled to [0, 1] 
   */
  public double angleChangeAt(double percentage) {
	//take atan of second derivative to get rotation rate of derivative in radians
    return ChezyMath.boundAngleNegPiToPiRadians(
            Math.atan(secondDerivativeAt(percentage)));
  }
  
  /**
   * Overrides toString to show a, b, c, d, and e values
   */
  public String toString() {
    return "a=" + a_ + "; b=" + b_ + "; c=" + c_ + "; d=" + d_ + "; e=" + e_;
  }
  
  //bob was here
}
