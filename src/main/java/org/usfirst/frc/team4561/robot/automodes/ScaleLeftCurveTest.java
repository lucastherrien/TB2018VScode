package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.ElevatorGroundPosition;
import org.usfirst.frc.team4561.robot.commands.ElevatorScalePosition;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ScaleLeftCurveTest extends CommandGroup {
	double delay = Robot.oi.getDashboardDelaySlider();

	public ScaleLeftCurveTest(){
		addSequential(new SpeedGear());
    	addSequential(new ArmVertical());
    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    	addSequential(new DriveMagic((int) (288+(RobotMap.DRIVETRAIN_CIRCUMFERENCE*1/4)), 288)); // forward
		addSequential(new DriveMagic(10,10)); // forward
		addSequential(new ElevatorScalePosition()); // elevate
		addSequential(new DriveMagic(10,10));
		addSequential(new ArmReleasePosition());
		addSequential(new IntakeRelease()); // drop power cube
		addSequential(new ArmVertical());
		addSequential(new DriveMagic(-20, -20));
		addSequential(new ElevatorGroundPosition());
    	
	}
}
