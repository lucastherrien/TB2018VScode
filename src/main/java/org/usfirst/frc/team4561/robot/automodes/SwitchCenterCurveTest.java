package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class SwitchCenterCurveTest extends CommandGroup {

	double delay = Robot.oi.getDashboardDelaySlider();
	
	public SwitchCenterCurveTest(){
		addSequential(new SpeedGear());
    	addSequential(new ArmVertical());
    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    	
    	// get side of switch from FMS
    	//addSequential(new CheckSwitchSide());
    	// on the right
    	if (Robot.switchFMSSideRight) {
    		addSequential(new DriveMagic(5, (int) (5+(RobotMap.DRIVETRAIN_CIRCUMFERENCE*55/360))));
    		addSequential(new DriveMagic((int) (71+(RobotMap.DRIVETRAIN_CIRCUMFERENCE*55/360)), 71)); // forward
    		addSequential(new DriveMagic(23, 23)); // forward
    		addSequential(new ArmReleasePosition());
    		//addSequential(new DriveMagic(10,10));
    		addSequential(new IntakeRelease()); // drop power cube
    	}
    	// on the left
    	else {
    		addSequential(new DriveMagic((int) (5+(RobotMap.DRIVETRAIN_CIRCUMFERENCE*55/360)), 5)); // forward	
    		addSequential(new DriveMagic(71, (int) (71+(RobotMap.DRIVETRAIN_CIRCUMFERENCE*55/360)))); // forward
    		addSequential(new DriveMagic(23, 23)); // forward
    		addSequential(new ArmReleasePosition());
    		//addSequential(new DriveMagic(10,10));
    		addSequential(new IntakeRelease()); // drop power cube
    	}
    }
}
