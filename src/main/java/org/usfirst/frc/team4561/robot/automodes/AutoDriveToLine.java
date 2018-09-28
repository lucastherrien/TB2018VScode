package org.usfirst.frc.team4561.robot.automodes;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.usfirst.frc.team4561.robot.commands.TorqueGear;
import org.usfirst.frc.team4561.robot.commands.TurnMagic;
import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
/**
 * This auto mode drives past the auto line if the robot is on the left or right side of the field.
 * @author Ben
 */
public class AutoDriveToLine extends CommandGroup {
	
	double delay = Robot.oi.getDashboardDelaySlider();
	
	public AutoDriveToLine() {
		// torque mode
		addSequential(new SpeedGear());
		addSequential(new ArmVertical());
		// wait preassigned time
    	addSequential(new WaitCommand(delay));
		// cross the auto line and score points
    	//System.out.println("Running");
		addSequential(new DriveMagic(150, 150));
//		addSequential(new WaitCommand(3));
		//addSequential(new TankDriveTimed(1, 1, 1));
		// addSequential(new WaitCommand(3));
		// done
	}
}
