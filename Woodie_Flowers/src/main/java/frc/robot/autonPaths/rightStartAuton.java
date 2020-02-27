/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.TurnDotEXE;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.trajectories.ThreeMetersBack;
import frc.robot.trajectories.rightDist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class rightStartAuton extends SequentialCommandGroup {
  static LimelightSubsystem m_limelightSubsystem;
  static DrivetrainSubsystem m_drivetrainSubsystem;
  static Shooter m_shooter;
  /**
   * Creates a new rightStartAuton.
   */
  /*public rightStartAuton() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new TurnDotEXE(m_drivetrainSubsystem, -90, 3), new rightDist().ramseteCommand, new TurnDotEXE(m_drivetrainSubsystem, 90, 3), new ThreeMetersBack().ramseteCommand, new AlignAndShootCommand(m_limelightSubsystem, m_drivetrainSubsystem, m_shooter));
  }*/
}
