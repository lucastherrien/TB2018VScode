package org.usfirst.frc.team4561.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *This is the IntakeRelease command
 *@author karth
 */
public class IntakeInJig extends CommandGroup {
	public IntakeInJig(){
	addSequential(new IntakeLeft());
	addSequential(new IntakeRight());
	}
}
