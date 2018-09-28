package org.usfirst.frc.team4561.robot.automodes;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.usfirst.frc.team4561.robot.commands.TurnMagic;
import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
/**
 * This auto mode drives past the auto line if the robot is in the center of the field.
 * This is currently in progress.
 * @author Ben
 */
public class AutoDriveToLineCenter extends CommandGroup {
	
	double delay = Robot.oi.getDashboardDelaySlider();
	
	public AutoDriveToLineCenter() {
		// torque mode
		addSequential(new SpeedGear());
		// wait preassigned time
    	addSequential(new WaitCommand(delay));
		// forward
		addSequential(new DriveMagic(50, 50));
		// turn right
		addSequential(new TurnMagic(90));
		// forward
		addSequential(new DriveMagic(50, 50));
		// turn left
		addSequential(new TurnMagic(-90));
		// cross the line and score points
		addSequential(new DriveMagic(50, 50));
		// addSequential(new WaitCommand(3));
		// done
	}
}
