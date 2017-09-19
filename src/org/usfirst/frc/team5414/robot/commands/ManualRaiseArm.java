package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualRaiseArm extends Command {

    public ManualRaiseArm() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.geararm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.gearcollector.isLimitPressed())
    	{
    		Robot.geararm.stop();
//    		RobotMap.ArmPositionUp = Robot.geararm.currentposition();
//    		RobotMap.ArmPositionDown = RobotMap.ArmPositionUp + .33;
    		return;
    	}
    	Robot.geararm.raise(RobotMap.SlowArmSpeedUp);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.geararm.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}