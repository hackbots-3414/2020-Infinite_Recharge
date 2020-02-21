/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GreenLine extends SequentialCommandGroup {
    public GreenLine(DrivetrainSubsystem drive) {
        addCommands(
        new Forward(drive),
        new Left_Turn(drive),
        new Forward(drive),
        new Forward(drive),
        new Right_Turn(drive),
        new GreenRight(drive),
        new Backwards(drive),
        new GreenBackwards(drive)


        );
        
    }
}
