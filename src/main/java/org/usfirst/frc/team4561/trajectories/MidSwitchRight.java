package org.usfirst.frc.team4561.trajectories;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

/**
 * Trajectory from middle starting position to the right side of the switch.
 * A copy of MidSwitchLeft but with waypoints changed.
 * @author Ben
 */
public class MidSwitchRight extends Path {
	
	// ALL units are in feet. Commands convert them to TalonSRX ticks later.
	
    public MidSwitchRight() {
    	
    	// Create waypoints (knots of the Hermite spline). Units are in feet.
    	// First point is the starting position, last point is the end.
    	// Angles are in radians, positive Y is to the left, positive X is forward
    	points = new Waypoint[] {
    			new Waypoint(3.22, 13.23, 0),
    			new Waypoint(6.00, 11.0, Pathfinder.d2r(-45)),
    			new Waypoint(11.67, 9.04, 0)
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
