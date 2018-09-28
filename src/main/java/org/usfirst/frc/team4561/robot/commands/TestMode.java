package org.usfirst.frc.team4561.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4561.robot.Robot;


public class TestMode extends Command{
    

    int motor;
    
    public TestMode() {
        motor = (int)Robot.oi.getDashboardSlider3();
        switch (motor) {
        case 0:
            requires(Robot.driveTrain);
        case 1:
            requires(Robot.driveTrain);

        case 2:
            requires(Robot.elevator);
            break;
        case 3:
            requires(Robot.elevator);
        case 4:
            requires(Robot.intake);
        case 5:
            requires(Robot.intake);
        
            break;
        }
    }    
        protected void execute() {
            switch (motor) {
                case 0:
                    Robot.driveTrain.setRightMotorPower(Robot.oi.getLeftStickX());
                case 1:
                    Robot.driveTrain.setLeftMotorPower(Robot.oi.getLeftStickX());
                case 2:
                    Robot.elevator.setPowerOne(Robot.oi.getLeftStickX());
                case 3:    
                    Robot.elevator.setPowerTwo(Robot.oi.getLeftStickX());
                case 4:
                    Robot.intake.setLeftIntake(Robot.oi.getLeftStickX());    
                case 5:
                    Robot.intake.setRightIntake(Robot.oi.getLeftStickX());
            }
            
        }
        protected void initialize() {
                            
     }            
            
        
         protected void end() {
                switch (motor) {
                    case 0:
                        Robot.driveTrain.setLeftMotorPower(0);
                    case 1:
                        Robot.driveTrain.setRightMotorPower(0);
                    case 2:
                        Robot.elevator.setPowerOne(0);
                    case 3:
                        Robot.elevator.setPowerOne(0);    
                    case 4:
                        Robot.intake.setLeftIntake(0);
                    case 5:
                        Robot.intake.setRightIntake(0);
                }
        
        
    }

    
    protected boolean isFinished() {
        
        return false;
    }
    
    
    protected void interrupted() {
        end();
    }
    
    
    
    
}
