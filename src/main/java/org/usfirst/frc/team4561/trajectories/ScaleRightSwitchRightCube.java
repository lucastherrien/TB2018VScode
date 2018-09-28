package org.usfirst.frc.team4561.trajectories;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

/**
 * Trajectory from the ending point of ScaleRightTurnAround to the second to right cube at the switch.
 * @author Ben
 */
public class ScaleRightSwitchRightCube extends Path {
	
	// ALL units are in feet. Commands convert them to TalonSRX ticks later.
	
    public ScaleRightSwitchRightCube() {
    	
    	// Create waypoints (knots of the Hermite spline). Units are in feet.
    	// First point is the starting position, last point is the end.
    	// Angles are in radians, positive Y is to the left, positive X is forward
    	points = new Waypoint[] {
    			new Waypoint(22, 3, Pathfinder.d2r(90)),
    			new Waypoint(17, 9.99, Pathfinder.d2r(180))
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
