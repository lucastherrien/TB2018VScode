package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMagic extends Command {

	int left;
	int right;
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.driveTrain.nearGoal();
	}
	
	public DriveMagic(int left, int right){
		this.left = left;
		this.right = right;
	}
	
	protected void initialize(){
		Robot.driveTrain.setSensorPhase(RobotMap.LEFT_SIDE_SENSOR_PHASE_REVERSED, RobotMap.RIGHT_SIDE_SENSOR_PHASE_REVERSED);
		Robot.driveTrain.magicDrive(left, right);

	}
	protected void execute(){
		
		//System.out.println("Running");
			
			
		SmartDashboard.putNumber("Left Speed", Robot.driveTrain.getLeftSpeed());
		SmartDashboard.putNumber("Right Speed", Robot.driveTrain.getRightSpeed());
		SmartDashboard.putNumber("Left Pos", Robot.driveTrain.getLeftPos());
		SmartDashboard.putNumber("Right Pos", Robot.driveTrain.getRightPos());
		SmartDashboard.putNumber("Left Error", Robot.driveTrain.getLeftError());
		SmartDashboard.putNumber("Right Error", Robot.driveTrain.getRightError());
		SmartDashboard.putNumber("Gyro Angle", Robot.driveTrain.getGyroAngle());
		SmartDashboard.putNumber("Gyro Rate", Robot.driveTrain.getGyroRate());
		SmartDashboard.putNumber("Correction Value", 0);
		SmartDashboard.putNumber("Avg Speed", Robot.driveTrain.avgSpeed());


	}
}
