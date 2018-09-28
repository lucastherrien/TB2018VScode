package org.usfirst.frc.team4561.robot.triggers;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.buttons.Trigger;

public class ToggleElevatorPIDTrigger extends Trigger {

	@Override
	public boolean get() {
		return (Robot.oi.getControllerButton(RobotMap.TOGGLE_PID_BUTTON_ONE)||Robot.oi.getControllerButton(RobotMap.TOGGLE_PID_BUTTON_TWO))&&Robot.oi.getControllerButton(RobotMap.TOGGLE_ELEVATOR_BUTTON);
	}

}
