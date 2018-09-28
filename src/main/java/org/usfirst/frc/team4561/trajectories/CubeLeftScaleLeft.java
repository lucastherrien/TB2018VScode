package org.usfirst.frc.team4561.trajectories;
import org.usfirst.frc.team4561.robot.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

/**
 * From left scale to left cube.
 * @author Kaiz
 *
 */
public class CubeLeftScaleLeft extends Path {
	
	// ALL units are in feet. Commands convert them to TalonSRX ticks later.
	
    public CubeLeftScaleLeft() {
    	
    	// Create waypoints (knots of the Hermite spline). Units are in feet.
    	// First point is the starting position, last point is the end.
    	// Angles are in radians, positive Y is to the left, positive X is forward
    	points = new Waypoint[] {
    			new Waypoint(18, 19.6, Pathfinder.d2r(20)),
    			new Waypoint(21.86, 18, 0)
    		};
    	reverse = false;
    	//this.config.max_velocity = RobotMap.WHEELBASE_WIDTH_SCALE;
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
