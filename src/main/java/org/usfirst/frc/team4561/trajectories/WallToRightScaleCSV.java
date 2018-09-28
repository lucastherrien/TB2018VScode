package org.usfirst.frc.team4561.trajectories;
import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class WallToRightScaleCSV extends Path {
	Trajectory left;
    Trajectory right;
	public void TwoCubeAutoTranjectory(){
		//assign trajectories to their respective files
		File leftFile = new File("wall to scale right_left_detailed.csv");
		left = Pathfinder.readFromCSV(leftFile);
		File rightFile = new File("wall to scale right_left_detailed.csv");
		right = Pathfinder.readFromCSV(rightFile);
	}
	
	public Trajectory getLeftTrajectory() {
		return left;
	}

	public Trajectory getRightTrajectory() {
		return right;
	}
}
