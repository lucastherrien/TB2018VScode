package org.usfirst.frc.team4561.trajectories;

import org.usfirst.frc.team4561.robot.OI;
import org.usfirst.frc.team4561.robot.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class Path {
	// Sample Counts: SAMPLES_HIGH (100 000), SAMPLES_LOW  (10 000), SAMPLES_FAST (1 000)
	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_FAST,
			RobotMap.TIME_STEP, RobotMap.MAX_VELOCITY, RobotMap.MAX_ACCELERATION, RobotMap.MAX_JERK);
	
	Waypoint[] points;
    Trajectory trajectory;
    TankModifier modifier;
    Trajectory left;
    Trajectory right;
    double[][] leftArray;
    double[][] rightArray;
    boolean reverse;
    double wheelbase = RobotMap.WHEELBASE_WIDTH_SWITCH;
	
    public void generateTrajectoriesAndArrays() {
    	trajectory = Pathfinder.generate(points, config);
    	// Wheelbase Width (feet)
    	double wheelbaseWidth = RobotMap.UNITS_PER_10_ROBOT_REVOLUTIONS / 10.0;
    	wheelbaseWidth /= Math.PI; // Diameter in units
    	wheelbaseWidth = OI.units2Ft(wheelbaseWidth);
    	modifier = new TankModifier(trajectory).modify(wheelbase); // 1.865 (1.375 for kongo)
    	// Do something with the new Trajectories...
    	left = modifier.getLeftTrajectory();
    	right = modifier.getRightTrajectory();
    	leftArray = new double[left.length()][2];
    	for (int i = 0; i < left.length(); i++) {
    		Trajectory.Segment seg = left.get(i);
    		double[] point = {seg.position, seg.velocity, seg.dt};
    		leftArray[i] = point;
    	}
    	rightArray = new double[right.length()][2];
    	for (int i = 0; i < right.length(); i++) {
    		Trajectory.Segment seg = right.get(i);
    		double[] point = {seg.position, seg.velocity, seg.dt};
    		rightArray[i] = point;
    	}
    }
    
	public Trajectory getLeftTrajectory() {
		return left;
	}
	
	public double[][] getLeftArray() {
		return leftArray;
	}

	public Trajectory getRightTrajectory() {
		return right;
	}
	
	public double[][] getRightArray() {
		return rightArray;
	}
	
	/**
	 * Get 0-based number of indices in arrays.
	 * @return
	 */
	public int getCount() {
		return leftArray.length;
	}
	
	public boolean isReversed() {
		return reverse;
	}
	
	public void printMainTrajectory() {
		System.out.format("%-9s\t", "Count");
		System.out.format("%-9s\t", "Time step");
		System.out.format("%-9s\t", "X");
		System.out.format("%-9s\t", "Y");
		System.out.format("%-9s\t", "Pos");
		System.out.format("%-9s\t", "Vel");
		System.out.format("%-9s\t", "Acc");
		System.out.format("%-9s\t", "Jerk");
		System.out.format("%-9s\t", "Heading");
		System.out.format("\n");
		for (int i = 0; i < trajectory.length(); i++) {
    		Trajectory.Segment seg = trajectory.get(i);
    		System.out.format("%-9s\t", i);
    		System.out.format("%-9s\t", seg.dt);
    		System.out.format("%-9s\t", seg.x);
    		System.out.format("%-9s\t", seg.y);
    		System.out.format("%-9s\t", seg.position);
    		System.out.format("%-9s\t", seg.velocity);
    		System.out.format("%-9s\t", seg.acceleration);
    		System.out.format("%-9s\t", seg.jerk);
    		System.out.format("%-9s\t", seg.heading);
    		System.out.format("\n");
		}
	}
	
	public void printLeftTrajectory() {
		System.out.format("%-9s\t", "Count");
		System.out.format("%-9s\t", "Time step");
		System.out.format("%-9s\t", "X");
		System.out.format("%-9s\t", "Y");
		System.out.format("%-9s\t", "Pos");
		System.out.format("%-9s\t", "Vel");
		System.out.format("%-9s\t", "Acc");
		System.out.format("%-9s\t", "Jerk");
		System.out.format("%-9s\t", "Heading");
		System.out.format("\n");
		for (int i = 0; i < left.length(); i++) {
    		Trajectory.Segment seg = left.get(i);
    		System.out.format("%-9s\t", i);
    		System.out.format("%-9s\t", seg.dt);
    		System.out.format("%-9s\t", seg.x);
    		System.out.format("%-9s\t", seg.y);
    		System.out.format("%-9s\t", seg.position);
    		System.out.format("%-9s\t", seg.velocity);
    		System.out.format("%-9s\t", seg.acceleration);
    		System.out.format("%-9s\t", seg.jerk);
    		System.out.format("%-9s\t", seg.heading);
    		System.out.format("\n");
		}
	}
	
	public void printRightTrajectory() {
		System.out.format("%-9s\t", "Count");
		System.out.format("%-9s\t", "Time step");
		System.out.format("%-9s\t", "X");
		System.out.format("%-9s\t", "Y");
		System.out.format("%-9s\t", "Pos");
		System.out.format("%-9s\t", "Vel");
		System.out.format("%-9s\t", "Acc");
		System.out.format("%-9s\t", "Jerk");
		System.out.format("%-9s\t", "Heading");
		System.out.format("\n");
		for (int i = 0; i < left.length(); i++) {
    		Trajectory.Segment seg = left.get(i);
    		System.out.format("%-9s\t", i);
    		System.out.format("%-9s\t", seg.dt);
    		System.out.format("%-9s\t", seg.x);
    		System.out.format("%-9s\t", seg.y);
    		System.out.format("%-9s\t", seg.position);
    		System.out.format("%-9s\t", seg.velocity);
    		System.out.format("%-9s\t", seg.acceleration);
    		System.out.format("%-9s\t", seg.jerk);
    		System.out.format("%-9s\t", seg.heading);
    		System.out.format("\n");
		}
	}
	
	public void printLeftArray() {
		System.out.format("%-9s\t", "Count");
		System.out.format("%-9s\t", "Pos");
		System.out.format("%-9s\t", "Vel");
		System.out.format("%-9s\t", "TimeDur");
		System.out.format("\n");
		for (int i = 0; i < leftArray.length; i++) {
    		System.out.format("%-9s\t", i);
    		System.out.format("%-9s\t", leftArray[i][0]);
    		System.out.format("%-9s\t", leftArray[i][1]);
    		System.out.format("%-9s\t", leftArray[i][2]);
    		System.out.format("\n");
		}
	}
	
	public void printRightArray() {
		System.out.format("%-9s\t", "Count");
		System.out.format("%-9s\t", "Pos");
		System.out.format("%-9s\t", "Vel");
		System.out.format("%-9s\t", "TimeDur");
		System.out.format("\n");
		for (int i = 0; i < rightArray.length; i++) {
    		System.out.format("%-9s\t", i);
    		System.out.format("%-9s\t", rightArray[i][0]);
    		System.out.format("%-9s\t", rightArray[i][1]);
    		System.out.format("%-9s\t", rightArray[i][2]);
    		System.out.format("\n");
		}
	}
}
