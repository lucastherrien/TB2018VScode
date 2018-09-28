package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.DriveProfile;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;

public class MotionProfileTestCSV extends CommandGroup {
	public MotionProfileTestCSV(){
		Trajectory pointsR = Robot.midScaleRightCSV.getRightTrajectory();
		Trajectory pointsL = Robot.midScaleRightCSV.getLeftTrajectory();
		addSequential(new DriveProfile(pointsR, pointsL));
	}
}
