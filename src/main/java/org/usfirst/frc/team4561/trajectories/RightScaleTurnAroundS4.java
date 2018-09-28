package org.usfirst.frc.team4561.trajectories;
import jaci.pathfinder.Waypoint;

/**
 * Trajectory from right scale position to pivot position.
 * @author Max
 *
 */
public class RightScaleTurnAroundS4 extends Path {
	
	// ALL units are in feet. Commands convert them to TalonSRX ticks later.
	
    public RightScaleTurnAroundS4() {
    	
    	// Create waypoints (knots of the Hermite spline). Units are in feet.
    	// First point is the starting position, last point is the end.
    	// Angles are in radians, positive Y is to the left, positive X is forward
    	points = new Waypoint[] {
    			new Waypoint(24.88, 6.8, 0),
    			    	new Waypoint(22, 3, 90)
    			};
    	reverse = false;
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
