package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.*;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * TerrorBytes proprietary code to place a cube on both the switch and the scale in auto
 * YAY
 * @author Ben (copy of Lucas's code except on the right)
 */
public class TwoCubeAutoRightPosition extends CommandGroup {
	
	double delay = Robot.oi.getDashboardDelaySlider();
	
    public TwoCubeAutoRightPosition() {
    	// get side of switch and scale from FMS
    	addSequential(new CheckSwitchSide());
    	addSequential(new CheckScaleSide());
    	// set transmission to low gear
    	addSequential(new TorqueGear());
    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    	// scale is on the right
    	if (Robot.scaleFMSSideRight) {
    		if(Robot.switchFMSSideRight){ // switch on the right
    			addSequential(new DriveMagic(100, 100)); // forward to scale
    			addSequential(new ElevatorScalePosition()); // elevator to scale position
            	addSequential(new ArmReleasePosition()); // prepare arm to drop cube on scale
            	addSequential(new IntakeRelease()); // bye bye power cube
            	addSequential(new TurnMagic(225)); // turn around
            	addSequential(new ElevatorGroundPosition()); // put the elevator down
            	addSequential(new DriveMagic(100, 100)); // forward to power cube
            	addSequential(new ArmIntakePosition()); //prepare arm to intake cube
            	addSequential(new IntakeIn()); // what? a new power cube? YAY
                addSequential(new DriveMagic(100, 100)); // forward to meet my friend, power cube
                addSequential(new ElevatorSwitchPosition()); // elevator to switch position
                addSequential(new ArmReleasePosition()); // arm to release position
                // ALL SYSTEMS GO FOR CUBE RELEASE
                addSequential(new IntakeRelease()); // and the moment you've all been waiting for... releasing the cube!
    			//done
    		} else { //switch on left
    			addSequential(new DriveMagic(100, 100)); // forward to scale
    			addSequential(new ElevatorScalePosition()); // elevator to scale position
            	addSequential(new ArmReleasePosition()); // prepare arm to drop cube on scale
            	addSequential(new IntakeRelease()); // bye bye power cube
            	addSequential(new DriveMagic(-100, -100)); // backward
            	addSequential(new TurnMagic(270)); // turn left
            	addSequential(new DriveMagic(100, 100)); // forward
            	addSequential(new TurnMagic(225)); // angle toward power cube
            	addSequential(new ElevatorGroundPosition()); // put the elevator down
            	addSequential(new DriveMagic(100, 100)); // forward to power cube
            	addSequential(new ArmIntakePosition()); //prepare arm to intake cube
            	addSequential(new IntakeIn()); // what? a new power cube? YAY
                addSequential(new DriveMagic(100, 100)); // forward to meet my friend, power cube
                addSequential(new ElevatorSwitchPosition()); // elevator to switch position
                addSequential(new ArmReleasePosition()); // arm to release position
                // ALL SYSTEMS GO FOR CUBE RELEASE
                addSequential(new IntakeRelease()); // and the moment you've all been waiting for... releasing the cube!
    			//done
    		}
    	}
    	//scale is on the left
    	else {
    		if(Robot.switchFMSSideRight){ //switch on right
    			addSequential(new DriveMagic(100, 100)); // forward
        		addSequential(new TurnMagic(270)); // turn left
        		addSequential(new DriveMagic(100, 100)); // forward
        		addSequential(new TurnMagic(90)); // turn right
        		addSequential(new DriveMagic(100, 100)); // forward to scale
    			addSequential(new ElevatorScalePosition()); // elevator to scale position
            	addSequential(new ArmReleasePosition()); // prepare arm to drop cube on scale
            	addSequential(new IntakeRelease()); // bye bye power cube
            	addSequential(new TurnMagic(180)); // turn around
            	addSequential(new DriveMagic(100, 100)); // forward
            	addSequential(new TurnMagic(270)); // turn left
            	addSequential(new DriveMagic(100, 100)); // forward
            	addSequential(new TurnMagic(90)); // turn right
        		addSequential(new DriveMagic(100, 100)); // forward to cube
        		addSequential(new TurnMagic(90)); // turn right
        		addSequential(new ElevatorGroundPosition()); // put the elevator down
            	addSequential(new DriveMagic(100, 100)); // forward to power cube
            	addSequential(new ArmIntakePosition()); //prepare arm to intake cube
            	addSequential(new IntakeIn()); // what? a new power cube? YAY
                addSequential(new DriveMagic(100, 100)); // forward to meet my friend, power cube
                addSequential(new DriveMagic(-100, -100)); // backward
                addSequential(new TurnMagic(270)); // turn left
                addSequential(new DriveMagic(100, 100)); // forward to switch
                addSequential(new TurnMagic(270)); // turn left
                addSequential(new DriveMagic(100, 100)); // forward to switch (last little bit)
                addSequential(new ElevatorSwitchPosition()); // elevator to switch position
                addSequential(new ArmReleasePosition()); // arm to release position
                // ALL SYSTEMS GO FOR CUBE RELEASE
                addSequential(new IntakeRelease()); // and the moment you've all been waiting for... releasing the cube!
    			//done
        	} else { //switch on left
        		addSequential(new DriveMagic(100, 100)); // forward
        		addSequential(new TurnMagic(270)); // turn left
        		addSequential(new DriveMagic(100, 100)); // forward
        		addSequential(new TurnMagic(90)); // turn right
        		addSequential(new DriveMagic(100, 100)); // forward to scale
    			addSequential(new ElevatorScalePosition()); // elevator to scale position
            	addSequential(new ArmReleasePosition()); // prepare arm to drop cube on scale
            	addSequential(new IntakeRelease()); // bye bye power cube
            	addSequential(new TurnMagic(180)); // turn around
            	addSequential(new ElevatorGroundPosition()); // put the elevator down
            	addSequential(new DriveMagic(100, 100)); // forward to power cube
            	addSequential(new ArmIntakePosition()); //prepare arm to intake cube
            	addSequential(new IntakeIn()); // what? a new power cube? YAY
                addSequential(new DriveMagic(100, 100)); // forward to meet my friend, power cube
                addSequential(new ElevatorSwitchPosition()); // elevator to switch position
                addSequential(new ArmReleasePosition()); // arm to release position
                // ALL SYSTEMS GO FOR CUBE RELEASE
                addSequential(new IntakeRelease()); // and the moment you've all been waiting for... releasing the cube!
        	}
    	}
    }
}