package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * This automode places a power cube on the scale if the robot is in the center of the Arcade.
 * This is currently in progress.
 * @author Ben
 */
public class AutoScaleCenterPosition extends CommandGroup {

	double delay = Robot.oi.getDashboardDelaySlider();
	
    public AutoScaleCenterPosition() {
    	addSequential(new TorqueGear());
    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
        // if our side is the right
    	// if our side is the right
    	if (Robot.scaleFMSSideRight) {
    		addSequential(new DriveMagic(50, 50)); // forward
    		addSequential(new TurnMagic(90)); // right
    		addSequential(new DriveMagic(72, 72)); // forward
    		addSequential(new TurnMagic(-90)); // left
    		addSequential(new DriveMagic(100, 100));
    		addSequential(new TurnMagic(-90));
    		addSequential(new DriveMagic(10, 10));
    		addSequential(new TurnMagic(90));
    		addSequential(new DriveMagic(90, 90)); // forward
    		addSequential(new ElevatorScalePosition());
    		addSequential(new DriveMagic(10,10));
    		addSequential(new ArmReleasePosition());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new ElevatorGroundPosition()); // put the elevator down
    	}
    	// if we are on the left
    	else {
    		addSequential(new DriveMagic(50, 50)); // forward
    		addSequential(new TurnMagic(-90)); // right
    		addSequential(new DriveMagic(72, 72)); // forward
    		addSequential(new TurnMagic(90)); // left
    		addSequential(new DriveMagic(100, 100));
    		addSequential(new TurnMagic(90));
    		addSequential(new DriveMagic(10, 10));
    		addSequential(new TurnMagic(-90));
    		addSequential(new DriveMagic(90, 90)); // forward
    		addSequential(new ElevatorScalePosition());
    		addSequential(new DriveMagic(10, 10));
    		addSequential(new ArmReleasePosition());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new ElevatorGroundPosition()); // put the elevator down
    	}
    }
}
