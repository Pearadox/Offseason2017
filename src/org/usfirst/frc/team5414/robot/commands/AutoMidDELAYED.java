package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMidDELAYED extends CommandGroup {

    public AutoMidDELAYED() {

    	addSequential(new ActivateTract());
    	
    	addSequential(new ShooterTriggerClose());
    	
    	addSequential(new DriveEncDist(-3.7));
    	
    	addSequential(new Delay(1));
    	
    	addSequential(new DriveEncDist(2));
    	
    	if(DriverStation.Alliance.Blue == DriverStation.getInstance().getAlliance())
    	{
    		addSequential(new RotateLeft(99.7));
    	}
    	else addSequential(new RotateRight(103.5));
    	
    	addParallel(new AutoShootOnly());
    	
    	addSequential(new DriveEncDist(-1.7, 6.9));
    	
    	addSequential(new Brake(1, .2));
    	
    	addSequential(new Delay(1));
    	
    	addSequential(new ShooterTriggerOpen());
    	
    }
}
