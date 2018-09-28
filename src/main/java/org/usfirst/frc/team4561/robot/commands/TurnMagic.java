package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class TurnMagic extends Command {

	public TurnMagic(double degrees){
		angle = degrees/360;
	}
	double angle;
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.driveTrain.nearGoal();
	}
	
	protected void initialize(){
		Robot.driveTrain.magicDrive((int) (RobotMap.DRIVETRAIN_CIRCUMFERENCE*angle), -(int) (RobotMap.DRIVETRAIN_CIRCUMFERENCE*angle));
	}

}
