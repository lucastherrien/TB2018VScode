package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;

public class DriveProfile extends Command {

	Trajectory pointsR;
	Trajectory pointsL;
	
	public DriveProfile(Trajectory pointsR, Trajectory pointsL){
		this.pointsL = pointsL;
		this.pointsR = pointsR;
	}
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}
	
	protected void initialize(){
		Robot.driveTrain.setUpMotionProfiling(pointsR, pointsL);
	}

	protected void execute(){
		Robot.driveTrain.runMotionProfile();
	}
	protected void end(){
		//Robot.driveTrain.holdMotionProfile();
	}
}
