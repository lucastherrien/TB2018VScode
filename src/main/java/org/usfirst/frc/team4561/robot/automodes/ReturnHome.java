package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.TurnMagic;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ReturnHome extends CommandGroup {
	public ReturnHome() {
		addSequential(new DriveMagic(10, 10));
		addSequential(new TurnMagic(180));
		addSequential(new DriveMagic(10, 10));
	}
}
