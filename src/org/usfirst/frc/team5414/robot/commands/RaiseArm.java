package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RaiseArm extends Command {

	double upPos = 0.0;
	double CurrentPos = 0;
	
    public RaiseArm() {
      
    	requires(Robot.geararm);
    }


    protected void initialize() {
//    	Robot.geararm.setPosition(RobotMap.ArmPositionUp);
    	setTimeout(1);
    }


    protected void execute() {
    	
    	Robot.geararm.raise(RobotMap.armRaiseSpeed);
//    	if (Robot.gearcollector.isLimitPressed())
//    	{
//    		Robot.geararm.stop();
//    		RobotMap.ArmPositionUp = Robot.geararm.currentposition();
//    		RobotMap.ArmPositionDown = RobotMap.ArmPositionUp + .33;
//    	}
   	}

    protected boolean isFinished() {

    	if(Robot.gearcollector.isLimitPressed() || isTimedOut())
    	{
    		Robot.geararm.stop();
    		return true;
    	}
    	return false;
    }

  
    protected void end() {
//    	Robot.geararm.setTalonMode();
//    	Robot.geararm.raise(upPos);						//after command has ended the gear arm motor is set to 0
    	Robot.geararm.stop();
    }


    protected void interrupted() {
//    	end();
    }
}
