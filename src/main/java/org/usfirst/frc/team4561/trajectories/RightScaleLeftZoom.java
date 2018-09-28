package org.usfirst.frc.team4561.trajectories;
import org.usfirst.frc.team4561.robot.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

/**
 * Trajectory from right starting position to the left side of the scale.
 * @author Ben
 */
public class RightScaleLeftZoom extends Path {
	
	// ALL units are in feet. Commands convert them to TalonSRX ticks later.
	
    public RightScaleLeftZoom() {
    	
    	// Create waypoints (knots of the Hermite spline). Units are in feet.
    	// First point is the starting position, last point is the end.
    	// Angles are in radians, positive Y is to the left, positive X is forward
    	points = new Waypoint[] {
    			new Waypoint(3.22, 3.58, 0),
    			new Waypoint(14, 3.58, Pathfinder.d2r(4)),
    			new Waypoint(19.5, 13.5, Pathfinder.d2r(90)),
    			new Waypoint(20, 19, Pathfinder.d2r(70)),
    			new Waypoint(24.97, 20.2, 0)
    	};
    	reverse = false;
    	this.wheelbase = RobotMap.WHEELBASE_WIDTH_SCALE;
    	this.config.max_velocity = 5;
    	this.config.max_acceleration = 2;
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
