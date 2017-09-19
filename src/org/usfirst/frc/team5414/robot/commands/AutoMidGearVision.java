package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMidGearVision extends CommandGroup {

    public AutoMidGearVision() {

    	addSequential(new ActivateTract());
    	
        addSequential(new RaiseArm());
        
    	addSequential(new DriveEncDistUntilVision(5));
    	
    	addSequential(new VisionScoringGroup());
//    	
//    	addSequential(new RotateLeft(90));
//    	
//    	addSequential(new DriveEncDist(5));
    }
}
