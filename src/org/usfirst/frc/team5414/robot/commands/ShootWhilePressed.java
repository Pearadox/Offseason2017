package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShootWhilePressed extends Command {

    public ShootWhilePressed() {
        requires(Robot.shooter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.shooter.shoot();
    	if(Robot.shooter.currentRPM() > .9 * RobotMap.ShooterRPM)
    	{
    		Robot.shooter.openTrigger();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !Robot.oi.stick.getRawButton(5);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}