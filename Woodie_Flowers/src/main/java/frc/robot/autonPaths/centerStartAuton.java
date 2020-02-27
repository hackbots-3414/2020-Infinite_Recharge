/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonPaths;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DriveConstants;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.TurnDotEXE;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.trajectories.ThreeMetersBack;
import frc.robot.trajectories.centerDist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class centerStartAuton extends SequentialCommandGroup {

  static LimelightSubsystem m_limelightSubsystem;
  static DrivetrainSubsystem m_drivetrainSubsystem;
  /**
   * Creates a new centerStartAuton.
   */
  /*public centerStartAuton() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new TurnDotEXE(m_drivetrainSubsystem, -90, 3), new RamseteCommand(TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), List.of(), new Pose2d(-1.4, 0, new Rotation2d(0)), new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
    DriveConstants.kMaxAccelerationMetersPerSecondSquared)
.setKinematics(DriveConstants.kDriveKinematics)
.addConstraint(new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      5))
.setReversed(true)), m_drivetrainSubsystem::getPose, disabledRamsete, new SimpleMotorFeedforward(DriveConstants.ksVolts,
    DriveConstants.kvVoltSecondsPerMeter,
    DriveConstants.kaVoltSecondsSquaredPerMeter), DriveConstants.kDriveKinematics, m_drivetrainSubsystem::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drivetrainSubsystem::tankDriveVolts, m_drivetrainSubsystem);, new TurnDotEXE(m_drivetrainSubsystem, 90, 3), new ThreeMetersBack(), new AlignAndShootCommand(m_limelightSubsystem, m_drivetrainSubsystem, m_shooter));
  }*/
}
