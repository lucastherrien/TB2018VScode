package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeDrive extends Command {

	public IntakeDrive(){
		requires(Robot.intake);
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	protected void execute(){
		if (Robot.oi.getRightButton(RobotMap.INTAKE_BUTTON)){
			Robot.intake.intakeIn();
		}
		else if (Robot.oi.getLeftButton(RobotMap.OUTTAKE_FULL_BUTTON)){
			Robot.intake.release();
		}
		else if (Robot.oi.getLeftButton(RobotMap.RELEASE_BUTTON)){
			Robot.intake.intakeOutHalf();
		}
		else if (Robot.oi.getLeftButton(RobotMap.OUTTAKE_WEAK_BUTTON)) {
			Robot.intake.intakeOutWeak();
		}
		else if (Robot.oi.getControllerButton(RobotMap.INTAKE_INFINITE_BUTTON)) {
			Robot.intake.intakeForever();
		}
		else {
			if (Robot.oi.getControllerButton(5)) {
				Robot.intake.set(-Robot.oi.getControllerLTrigger(), -Robot.oi.getControllerRTrigger());
			} else {
				Robot.intake.set(Robot.oi.getControllerLTrigger(), Robot.oi.getControllerRTrigger());
			}
		}
		
		if (Robot.oi.getLeftButton(1)) {
			Robot.intake.bobClawOpen();
		}
		else { 
			Robot.intake.bobClawClose();
		}
	}
}
