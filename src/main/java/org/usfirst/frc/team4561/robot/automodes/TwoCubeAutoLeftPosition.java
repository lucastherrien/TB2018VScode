package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.*;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * TerrorBytes proprietary code to place a cube on both the switch and the scale in auto
 * Suck it 900
 * @author Lucas
 */
public class TwoCubeAutoLeftPosition extends CommandGroup {
	
	double delay = Robot.oi.getDashboardDelaySlider();
	
    public TwoCubeAutoLeftPosition() {
    	// get side of switch and scale from FMS
    	addSequential(new CheckSwitchSide());
    	addSequential(new CheckScaleSide());
    	// set transmission to low gear
    	addSequential(new TorqueGear());
    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    	// scale is on the right
    	if (Robot.scaleFMSSideRight) {
    		if(Robot.switchFMSSideRight){
    			addSequential(new DriveMagic(50, 50)); // forward
        		addSequential(new TurnMagic(90)); // turn right
        		addSequential(new DriveMagic(36, 36)); // forward
        		addSequential(new TurnMagic(-90)); // turn left
        		addSequential(new DriveMagic(40, 40)); // forward
        		addSequential(new ArmReleasePosition());
        		addSequential(new DriveMagic(10,10));
        		addSequential(new IntakeRelease()); // drop power cube
        		
    		addSequential(new DriveMagic(-50, -50)); // back up a tad so we can align to intake the second cube
    		addSequential(new TurnMagic(-90));
    		addSequential(new DriveMagic(25,25));
    		addSequential(new TurnMagic(90));
    		
    		addSequential(new TurnMagic(315)); // align ourselves to intake the second cube
    		addSequential(new ElevatorGroundPosition()); //elevator to ground position
    		addSequential(new ArmIntakePosition()); //prepare arm to intake cube
    		addSequential(new IntakeIn()); // spin up intake mech to intake cube
    		addSequential(new DriveMagic(100, 100)); // move forward to intake cube
    		addSequential(new DriveMagic(-100, -100)); // back up a tad
    		addSequential(new TurnMagic(315)); // turn left 45 degrees
    		addSequential(new DriveMagic(100, 100)); // move forward
    		addSequential(new TurnMagic(270)); // turn left to face the switch
    		addSequential(new ElevatorScalePosition()); //elevator to scale position
    		addSequential(new ArmReleasePosition()); //prepare arm to drop cube on scale
    		addSequential(new DriveMagic(100, 100)); // move forward to scale
    		addSequential(new IntakeRelease()); // drop power cube on scale
    		//done
    		} else { //switch on right
    		addSequential(new DriveMagic(100, 100)); // forward to turning point
        	addSequential(new TurnMagic(90)); // turn right to face platform zone
        	addSequential(new DriveMagic(100, 100)); // forward to left side of switch, SHORTER DISTANCE THAN ABOVE SEQUENCE
        	addSequential(new TurnMagic(90)); // turn right to face the switch
        	addSequential(new DriveMagic(100, 100)); // forward to drop cube on switch
        	addSequential(new ElevatorSwitchPosition()); //elevator to switch position
        	addSequential(new ArmReleasePosition()); //prepare arm to drop cube on switch
        	addSequential(new IntakeRelease()); // drop power cube on switch
        	addSequential(new DriveMagic(-100, -100)); // back up a tad so we can align to intake the second cube
        	addSequential(new TurnMagic(315)); // align ourselves to intake the second cube
        	addSequential(new ElevatorGroundPosition()); //elevator to ground position
        	addSequential(new ArmIntakePosition()); //prepare arm to intake cube
        	addSequential(new IntakeIn()); // spin up intake mech to intake cube
        	addSequential(new DriveMagic(100, 100)); // move forward to intake cube
        	addSequential(new DriveMagic(-100, -100)); // back up a tad
        	addSequential(new TurnMagic(315)); // turn left 45 degrees
        	addSequential(new DriveMagic(100, 100)); // move forward, LONGER DISTANCE THAN ABOVE SEQUENCE
        	addSequential(new TurnMagic(270)); // turn left to face the switch
        	addSequential(new ElevatorScalePosition()); //elevator to scale position
        	addSequential(new ArmReleasePosition()); //prepare arm to drop cube on scale
        	addSequential(new DriveMagic(100, 100)); // move forward to scale
        	addSequential(new IntakeRelease()); // drop power cube on scale
        	//done
    		}
    	}
    	//scale is on the left
    	else {
    		if(Robot.switchFMSSideRight){ //switch on right
        	addSequential(new DriveMagic(100, 100)); // forward ~3/4 of the way towards the scale
        	addSequential(new ElevatorScalePosition()); // elevator to scale position
        	addSequential(new ArmReleasePosition()); // prepare arm to drop cube on scale
        	addSequential(new DriveMagic(100, 100)); // forward to scale
        	addSequential(new IntakeRelease()); // drop power cube
        	addSequential(new DriveMagic(-100, -100)); // back up a tad
        	addSequential(new ElevatorGroundPosition()); //elevator to ground position
            addSequential(new ArmIntakePosition()); //prepare arm to intake cube
            addSequential(new DriveMagic(-100, -100)); // back up a tad more
            addSequential(new TurnMagic(90)); // turn right to face platform zone
            addSequential(new DriveMagic(100, 100)); // forward to align to intake second cube
            addSequential(new TurnMagic(45)); // turn right to face second cube
            addSequential(new IntakeIn()); // spin up intake mech to intake cube
            addSequential(new DriveMagic(100, 100)); // forward to intake second cube
            addSequential(new TurnMagic(315)); // align ourselves to the switch
            addSequential(new IntakeRelease()); // drop power cube on switch
            //done
        	} else { //switch on left
        	addSequential(new DriveMagic(100, 100)); // forward ~3/4 of the way towards the scale
            addSequential(new ElevatorScalePosition()); // elevator to scale position
            addSequential(new ArmReleasePosition()); // prepare arm to drop cube on scale
            addSequential(new DriveMagic(100, 100)); // forward to scale
            addSequential(new IntakeRelease()); // drop power cube
            addSequential(new DriveMagic(-100, -100)); // back up a tad
            addSequential(new ElevatorGroundPosition()); //elevator to ground position
            addSequential(new ArmIntakePosition()); //prepare arm to intake cube
            addSequential(new DriveMagic(-100, -100)); // back up a tad more
            addSequential(new TurnMagic(90)); // turn right to face platform zone
            addSequential(new DriveMagic(100, 100)); // forward to align to intake second cube, SHORTER THAN ABOVE SEQUENCE
            addSequential(new TurnMagic(45)); // turn right to face second cube
            addSequential(new IntakeIn()); // spin up intake mech to intake cube
            addSequential(new DriveMagic(100, 100)); // forward to intake second cube
            addSequential(new TurnMagic(315)); // align ourselves to the switch
            addSequential(new IntakeRelease()); // drop power cube on switch
        	}
    	}
    }
}