package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class TurnGyro extends Command {
	
	double target;
	public TurnGyro(double angle) {
		target = angle;
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}
	
	protected void excecute() {
		Robot.driveTrain.goToAngle(target);
	}

}
