package org.usfirst.frc.team4561.trajectories;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class MidCubePileReverse extends Path {
	public MidCubePileReverse() {
		// Create waypoints (knots of the Hermite spline). Units are in feet.
    	// First point is the starting position, last point is the end.
    	// Angles are in radians, positive Y is to the left, positive X is forward
    	points = new Waypoint[] {
    			new Waypoint(9, 13.5, Pathfinder.d2r(-20)),
    			new Waypoint(3.22, 13.23, 0)
    	};
    	reverse = true;
    	this.config.max_velocity = 13.9;
    	this.config.max_acceleration = 6;
    	this.wheelbase = 3.1;
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
