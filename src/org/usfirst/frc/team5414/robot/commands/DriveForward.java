package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveForward extends CommandGroup {

    public DriveForward() {
        
    	addSequential(new DriveEncDist(1));
    	
    }
}
