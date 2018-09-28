package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class ToggleElevatorPID extends Command {

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}
	
	protected void execute(){
		RobotMap.ELEVATOR_PID = !RobotMap.ELEVATOR_PID;
		Robot.elevator.setGoalRelative((int) Robot.elevator.getElevatorPos());
		System.out.println("Toggled Elevator");
	}

}
