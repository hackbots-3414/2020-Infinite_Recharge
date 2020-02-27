/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class rightDist extends CommandBase {
  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  TrajectoryConfig configReversedTrue;
  TrajectoryConfig configReversedFalse;
  String trajectoryJSON = "paths/center_auton_start.wpilib.json";
  Path trajectoryPath;
  Trajectory trajectory;
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  RamseteController disabledRamsete = new RamseteController() {
    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
            double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
    }
};
  public RamseteCommand ramseteCommand = new RamseteCommand(TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), List.of(), new Pose2d(-3, 0, new Rotation2d(0)), configReversedTrue), m_drivetrainSubsystem::getPose, disabledRamsete, new SimpleMotorFeedforward(DriveConstants.ksVolts,
    DriveConstants.kvVoltSecondsPerMeter,
    DriveConstants.kaVoltSecondsSquaredPerMeter), DriveConstants.kDriveKinematics, m_drivetrainSubsystem::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drivetrainSubsystem::tankDriveVolts, m_drivetrainSubsystem);
  /**
   * Creates a new sixMeterTrajectory.
   */
  public rightDist() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      5);
  configReversedTrue =
      new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                           DriveConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint)
          .setReversed(true);
  configReversedFalse =
  new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
  DriveConstants.kMaxAccelerationMetersPerSecondSquared)
// Add kinematics to ensure max speed is actually obeyed
.setKinematics(DriveConstants.kDriveKinematics)
// Apply the voltage constraint
.addConstraint(autoVoltageConstraint)
.setReversed(false);
          // An example trajectory to follow.  All units in meters.
          try {
            trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          
          } catch (IOException ex) {
            // TODO Auto-generated catch block
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            ex.printStackTrace();
          }
      
          Trajectory backwards = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), List.of(), new Pose2d(-3, 0, new Rotation2d(0)), configReversedTrue);
            RamseteController disabledRamsete = new RamseteController() {
              @Override
              public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                      double angularVelocityRefRadiansPerSecond) {
                  return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
              }
          };
/*
       RamseteCommand ramseteCommand = new RamseteCommand(
          backwards,
          m_drivetrainSubsystem::getPose,
          disabledRamsete,
         //new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                     DriveConstants.kvVoltSecondsPerMeter,
                                     DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          m_drivetrainSubsystem::getWheelSpeeds,
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drivetrainSubsystem::tankDriveVolts,
          m_drivetrainSubsystem/
      );
*/
      
      
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
