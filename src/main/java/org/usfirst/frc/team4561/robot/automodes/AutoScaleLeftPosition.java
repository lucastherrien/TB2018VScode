package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.ElevatorGroundPosition;
import org.usfirst.frc.team4561.robot.commands.ElevatorScalePosition;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.TorqueGear;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.TurnMagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * This automode places a power cube on the scale if the robot is on the left side of the Arcade.
 * This is currently in progress.
 * @author Ben
 */
public class AutoScaleLeftPosition extends CommandGroup {

	double delay = Robot.oi.getDashboardDelaySlider();
	
    public AutoScaleLeftPosition() {
    	
    	addSequential(new TorqueGear());
    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
   
    	 // if our side is the left
    	// if our side is the left
    	if (!(Robot.scaleFMSSideRight)) {
    		addSequential(new DriveMagic(312, 312)); // forward
    		addSequential(new TurnMagic(90)); // right
    		addSequential(new DriveMagic(10,10)); // forward
    		addSequential(new ElevatorScalePosition()); // elevate
    		addSequential(new DriveMagic(10,10));
    		addSequential(new ArmReleasePosition());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new ArmVertical());
    		addSequential(new DriveMagic(-20, -20));
    		addSequential(new ElevatorGroundPosition());
    	}
    	// else
    	else {
    		addSequential(new DriveMagic(200, 200)); // forward
    		addSequential(new TurnMagic(90)); // right
    		addSequential(new DriveMagic(337, 337)); // forward
    		addSequential(new TurnMagic(-90)); // left
    		addSequential(new DriveMagic(40, 40)); // forward
    		addSequential(new ElevatorScalePosition());
    		addSequential(new DriveMagic(10,10));
    		addSequential(new ArmReleasePosition());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new ElevatorGroundPosition()); // put the elevator down
    	}
    	
    }
}

