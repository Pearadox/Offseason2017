package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LowerArm extends Command {


    public LowerArm() {
    	requires(Robot.geararm);
    }

    
    protected void initialize() {
//    	Robot.geararm.setPosition(RobotMap.ArmPositionDown);
    	setTimeout(.75);
    }

  
    protected void execute() {
    	Robot.geararm.lower(RobotMap.armLowerSpeed);
    }

    
    protected boolean isFinished() {
        return isTimedOut();		//stops the command when the command times out
    }

 
    protected void end() {
    	Robot.geararm.stop();
    }

    
    protected void interrupted() {
    }
}
