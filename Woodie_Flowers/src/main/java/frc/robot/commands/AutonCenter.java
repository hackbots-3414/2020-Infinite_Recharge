/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.trajectories.ThreeMetersBack;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonCenter extends SequentialCommandGroup {
  private final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  /**
   * Creates a new AutonCenter.
   */
  public AutonCenter() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new ThreeMetersBack(), new TurnDotEXE(m_drivetrainSubsystem, 90, 1));
  }
}
