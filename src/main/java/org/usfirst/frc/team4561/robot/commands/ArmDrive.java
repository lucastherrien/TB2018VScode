package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class ArmDrive extends Command {

	public ArmDrive(){
		requires(Robot.arm);
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
	
	protected void execute(){
		int pov = Robot.oi.getControllerPOV();
		if (pov == RobotMap.ARM_DOWN_POV){
			Robot.arm.IntakePosition();
		}
		else if (pov == RobotMap.ARM_MIDDLE_POV1){
			Robot.arm.ReleasePosition();
		}
		else if (pov == RobotMap.ARM_MIDDLE_POV2){
			Robot.arm.DiagonalPosition();
		}
		else if (pov == RobotMap.ARM_UP_POV){
			Robot.arm.AnglePosition();
		}
		if (Robot.oi.getControllerLeftY() != 0) {
			double output = Math.copySign(Math.pow(Robot.oi.getControllerLeftY(), 4)/2, -Robot.oi.getControllerLeftY());
			if (output > 0 && output < 0.2) {
				output = 0.2;
			}
			Robot.arm.set(output);
			Robot.arm.resetBetter();
		}
		if (Robot.oi.getControllerLeftY() == 0 && RobotMap.ARM_PID && Robot.arm.isVelocity()) {
			Robot.arm.resetBetter();
			Robot.arm.setToGoal();
		}
//		if (Robot.arm.getEncoderVelocity() < 0){
//			Robot.arm.setEncoderPos((int) (Robot.arm.getEncoderPosition()+Robot.arm.getEncoderVelocity()));
//		}
	}
	
	protected void stop(){
		Robot.arm.stop();
	}
}
