/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ConveyorCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SequenceCommand extends SequentialCommandGroup {
  /**
   * Creates a new SequenceCommand.
   */
  public SequenceCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain, Shooter shooter,
      ConveyorSubsystem conveyor, StopCommand stop) {
    super();
    sequence(new AlignAndShootCommand(limelight, drivetrain, shooter), new ConveyorCommand(conveyor),
        new StopCommand(shooter, drivetrain));
  }
}
