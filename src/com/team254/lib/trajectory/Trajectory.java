package com.team254.lib.trajectory;

import com.team254.lib.util.ChezyMath;

/**
 * Implementation of a Trajectory using arrays as the underlying storage
 * mechanism.
 * 
 * This class stores a trajector as san array of "segments" which are essentially end-to-end
 * vectors with properties of:
 * 	-origin position
 * 	-velocity
 * 	-acceleration
 * 	-jerk
 * 	-heading
 * 	-dt
 * 	-x
 * 	-y
 * 
 * Trajectories are stored and are manipulatable through this class.
 *
 * @author Jared341
 */
public class Trajectory {
  
	/**
	 * This class encapsulates pairs of trajactories to be used together
	 * 
	 * They represent the trajectories for the left and right drivetrain.
	 */
  public static class Pair {
    public Pair(Trajectory left, Trajectory right) {
      this.left = left;
      this.right = right;
    }

    public Trajectory left;
    public Trajectory right;
  }

  /**
   * This class encapsulates segments, or discrete parts of a trajactory
   * 
   * It contrains every data point needed to drive along a specific part of a trajectory:
   * 
   * pos: x pos along spline
   * vel: velocity along segment
   * acc: acceleration along segment
   * jerk: jerk along segment
   * heading: rotational heading along segment
   * dt: amount of time along segment
   * x: x position of start of segment
   * y: y postition of start og segment
   *
   */
  public static class Segment {

    public double pos, vel, acc, jerk, heading, dt, x, y;

    public Segment() {
    }

    public Segment(double pos, double vel, double acc, double jerk,
            double heading, double dt, double x, double y) {
      this.pos = pos;
      this.vel = vel;
      this.acc = acc;
      this.jerk = jerk;
      this.heading = heading;
      this.dt = dt;
      this.x = x;
      this.y = y;
    }

    //copy all instance vars if another segment is passed as an arg
    public Segment(Segment to_copy) {
      pos = to_copy.pos;
      vel = to_copy.vel;
      acc = to_copy.acc;
      jerk = to_copy.jerk;
      heading = to_copy.heading;
      dt = to_copy.dt;
      x = to_copy.x;
      y = to_copy.y;
    }

    //print all instance vars for toString
    public String toString() {
      return "pos: " + pos + "; vel: " + vel + "; acc: " + acc + "; jerk: "
              + jerk + "; heading: " + heading;
    }
  }

  //declare instance vars: 
  //segments_, to hold all of the segments, 
  //and inverted_y_, to store if the function was flipped when the spline was transformed in the spline class
  Segment[] segments_ = null;
  boolean inverted_y_ = false;

  //declare empty tajectory with certain length of segments
  public Trajectory(int length) {
    segments_ = new Segment[length];
    for (int i = 0; i < length; ++i) {
      segments_[i] = new Segment();
    }
  }
  
  //decleare trajectory with pre-determined segments array
  public Trajectory(Segment[] segments) {
    segments_ = segments;
  }
  
  /**
   * setter for inverted_y_
   */
  public void setInvertedY(boolean inverted) {
    inverted_y_ = inverted;
  }

  /**
   * getter for the number of segments
   */
  public int getNumSegments() {
    return segments_.length;
  }

  /**
   * Getter for each segment by index.
   * 
   * Inverts the y value and the heading ig inverted_y_ is true.
   */
  public Segment getSegment(int index) {
    if (index < getNumSegments()) {
      if (!inverted_y_) {
        return segments_[index];
      } else {
        Segment segment = new Segment(segments_[index]);
        segment.y *= -1.0;
        segment.heading *= -1.0;
        return segment;
      }
    } else {
      return new Segment();
    }
  }
  
  /**
   * Setter for segments
   * 
   * Only works if index is on [0, segments_.length)
   */
  public void setSegment(int index, Segment segment) {
    if (index < getNumSegments()) {
      segments_[index] = segment;
    }
  }

  /**
   * Function for scaling pos, vel, acc, and jerk of all segments by a factor
   */
  public void scale(double scaling_factor) {
    for (int i = 0; i < getNumSegments(); ++i) {
      segments_[i].pos *= scaling_factor;
      segments_[i].vel *= scaling_factor;
      segments_[i].acc *= scaling_factor;
      segments_[i].jerk *= scaling_factor;
    }
  }

  /**
   * Appends segments of argument trajectory to the end of this trajectory
   */
  public void append(Trajectory to_append) {
    Segment[] temp = new Segment[getNumSegments()
            + to_append.getNumSegments()];

    for (int i = 0; i < getNumSegments(); ++i) {
      temp[i] = new Segment(segments_[i]);
    }
    for (int i = 0; i < to_append.getNumSegments(); ++i) {
      temp[i + getNumSegments()] = new Segment(to_append.getSegment(i));
    }

    this.segments_ = temp;
  }

  /**
   * Returns an identical copy of this trajectory (minus the value of y_inverted_)
   */
  public Trajectory copy() {
    Trajectory cloned
            = new Trajectory(getNumSegments());
    cloned.segments_ = copySegments(this.segments_);
    return cloned;
  }
  
  
  /**
   * Returns a copy of the Segment[] argument as a different object (no references)
   */
  private Segment[] copySegments(Segment[] tocopy) {
    Segment[] copied = new Segment[tocopy.length];
    for (int i = 0; i < tocopy.length; ++i) {
      copied[i] = new Segment(tocopy[i]);
    }
    return copied;
  }

  /**
   * Prints the index, pos, vel, acc, jerk, and heading data of each segment on a line for toString
   */
  public String toString() {
    String str = "Segment\tPos\tVel\tAcc\tJerk\tHeading\n";
    for (int i = 0; i < getNumSegments(); ++i) {
      Trajectory.Segment segment = getSegment(i);
      str += i + "\t";
      str += segment.pos + "\t";
      str += segment.vel + "\t";
      str += segment.acc + "\t";
      str += segment.jerk + "\t";
      str += segment.heading + "\t";
      str += "\n";
    }

    return str;
  }

  public String toStringProfile() {
    return toString();
  }

  /**
   * Prints the index, x, y, and heading data for each Segment for toStringEuclidean
   */
  public String toStringEuclidean() {
    String str = "Segment\tx\ty\tHeading\n";
    for (int i = 0; i < getNumSegments(); ++i) {
      Trajectory.Segment segment = getSegment(i);
      str += i + "\t";
      str += segment.x + "\t";
      str += segment.y + "\t";
      str += segment.heading + "\t";
      str += "\n";
    }

    return str;
  }
}
