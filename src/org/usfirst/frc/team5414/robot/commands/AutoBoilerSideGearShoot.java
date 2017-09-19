package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoBoilerSideGearShoot extends CommandGroup {

    public AutoBoilerSideGearShoot() {
    	
    	addSequential(new ShooterTriggerClose());
    	
    	if(DriverStation.Alliance.Blue == DriverStation.getInstance().getAlliance()) 
		{
    		addSequential(new DriveEncDist(-6.151678));
    		addSequential(new RotateRight(50));
		}
		else {   
			addSequential(new DriveEncDist(-6.051678));
    		addSequential(new RotateLeft(50));
		}
    	
    	addSequential(new DriveEncDist(-2.3, .6, 2));
    	addSequential(new Delay(.5));
		if(DriverStation.Alliance.Blue == DriverStation.getInstance().getAlliance()) 
		{
			addSequential(new DriveEncDist(3.9));
			addParallel(new AutoShootOnly());
    		addSequential(new RotateRight(129, .785));
		}
		else 
		{   
			addSequential(new DriveEncDist(3.9));
			addParallel(new AutoShootOnly());
    		addSequential(new RotateLeft(130.3847, .785));
    		
		}
		addSequential(new Delay(.3));
    	addSequential(new ShooterTriggerOpen());
    	
    }
}
