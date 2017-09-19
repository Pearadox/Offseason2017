package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMidGearShoot extends CommandGroup {

    public AutoMidGearShoot() {

    	addSequential(new ActivateTract());
    	
    	addSequential(new ShooterTriggerClose());
    	
    	addSequential(new DriveEncDist(-5.1, .7414));
    	
    	addSequential(new Drive(-.4, -.4));
    	
    	addSequential(new Delay(1.3));
    	
    	addSequential(new Drive(0, 0));
    	
    	addSequential(new DriveEncDist(2.9));
    	
    	if(DriverStation.Alliance.Blue == DriverStation.getInstance().getAlliance())
    	{
    		addSequential(new RotateLeft(99.7));
    	}
    	else addSequential(new RotateRight(119));
    	
    	addParallel(new AutoShootOnly());
    	
    	addSequential(new DriveEncDist(-1.7, 6.9));
    	
    	addSequential(new Delay(1));
    	
    	addSequential(new ShooterTriggerOpen());
    	
    }
}
