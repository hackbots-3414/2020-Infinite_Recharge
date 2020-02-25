/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.DriveConstants.kMaxAccelerationMetersPerSecondSquared;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DriveCommand m_driveCommand = new DriveCommand(m_drivetrainSubsystem);
  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  TrajectoryConfig config;
  String trajectoryJSON = "paths/center_auton_start.wpilib.json";
  Path trajectoryPath;
  Trajectory trajectory;
 
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSubsystem, m_driveCommand);
    configureButtonBindings();
  }
  public Command getAutonomousCommand() {
    
 
    
    autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      5);
  config =
      new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                           DriveConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint)
          .setReversed(true);
          // An example trajectory to follow.  All units in meters.
          try {
            trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          
          } catch (IOException ex) {
            // TODO Auto-generated catch block
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            ex.printStackTrace();
          }
          Trajectory backwardsCurve = TrajectoryGenerator.generateTrajectory(
            new Pose2d(6.724, -2.166, new Rotation2d(Math.PI)), List.of(), new Pose2d(2.732, -2.166, new Rotation2d(Math.PI)), config);
          Trajectory backwards = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), List.of(), new Pose2d(-6, 0, new Rotation2d(Math.PI / 2)), config);
          
          Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 0 ),
                new Translation2d(2, 0)),
            
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
            RamseteController disabledRamsete = new RamseteController() {
              @Override
              public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                      double angularVelocityRefRadiansPerSecond) {
                  return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
              }
          };

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
          m_drivetrainSubsystem

      );
      return ramseteCommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0));
      
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
