package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
import org.usfirst.frc.team4561.robot.commands.TurnMagic;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * This automode places a power cube on the switch if the robot is in the center of the arcade.
 * This is currently in progress.
 * @author Ben
 */
public class AutoSwitchCenterPosition extends CommandGroup {

	double delay = Robot.oi.getDashboardDelaySlider();
	
    public AutoSwitchCenterPosition() {
        
    	addSequential(new SpeedGear());
    	addSequential(new ArmVertical());
    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    	
    	// get side of switch from FMS
    	//addSequential(new CheckSwitchSide());
    	// on the right
    	if (Robot.switchFMSSideRight) {
    		addSequential(new DriveMagic(50, 50)); // forward
    		addSequential(new TurnMagic(-90)); // turn right
    		addSequential(new DriveMagic(50, 50)); // forward
    		addSequential(new TurnMagic(90)); // turn left
    		addSequential(new DriveMagic(34, 34)); // forward
    		addSequential(new ArmReleasePosition());
    		addSequential(new DriveMagic(10,10));
    		addSequential(new IntakeRelease()); // drop power cube
    	}
    	// on the left
    	else {
    		addSequential(new DriveMagic(50, 50)); // forward
    		addSequential(new TurnMagic(90)); // turn left
    		addSequential(new DriveMagic(50, 50)); // forward
    		addSequential(new TurnMagic(-90)); // turn right
    		addSequential(new DriveMagic(46, 46)); // forward
    		addSequential(new ArmReleasePosition());
    		addSequential(new DriveMagic(10,10));
    		addSequential(new IntakeRelease()); // drop power cube
    	}
    }
}
