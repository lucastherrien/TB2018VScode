package org.usfirst.frc.team4561.trajectories;
import org.usfirst.frc.team4561.robot.RobotMap;

import jaci.pathfinder.Waypoint;

/**
 * Trajectory from right starting position to the right side of the scale.
 * @author Max
 *
 */
public class RightScaleRight extends Path {
	
	// ALL units are in feet. Commands convert them to TalonSRX ticks later.
	
    public RightScaleRight() {
    	
    	// Create waypoints (knots of the Hermite spline). Units are in feet.
    	// First point is the starting position, last point is the end.
    	// Angles are in radians, positive Y is to the left, positive X is forward
    	points = new Waypoint[] {
    				new Waypoint(3.22, 3.58, 0),
    				new Waypoint(20, 5.76, 0),
    				new Waypoint(24.6, 6.90, 0)
    			};
    	reverse = false;
    	this.wheelbase = RobotMap.WHEELBASE_WIDTH_SCALE;
    	this.config.max_velocity = 3;
    	this.config.max_acceleration = 3;
    	generateTrajectoriesAndArrays();
    	
    	/* To print out points along trajectory...
    	 
	    	for (int i = 0; i < left.length(); i++) {
	    		Trajectory.Segment seg = trajectory.get(i);
	    
	    		System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
	        		seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
	            	seg.acceleration, seg.jerk, seg.heading);
			}
	
    	 */
    }
}
