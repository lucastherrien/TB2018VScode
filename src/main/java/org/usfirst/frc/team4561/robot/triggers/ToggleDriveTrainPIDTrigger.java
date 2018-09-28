package org.usfirst.frc.team4561.robot.triggers;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.buttons.Trigger;

public class ToggleDriveTrainPIDTrigger extends Trigger {

	@Override
	public boolean get() {
		return Robot.oi.getLeftButton(RobotMap.TOGGLE_DRIVETRAIN_BUTTON_ONE)&&Robot.oi.getLeftButton(RobotMap.TOGGLE_DRIVETRAIN_BUTTON_TWO)&&Robot.oi.getLeftButton(RobotMap.TOGGLE_DRIVETRAIN_BUTTON_THREE);
	}

}
