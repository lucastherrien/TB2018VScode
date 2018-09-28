package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SetGyro extends Command {
	double a;
	public SetGyro(double angle) {
		a = angle;
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}
	protected void execute() {
		Robot.gyro.set(a);
	}

}
