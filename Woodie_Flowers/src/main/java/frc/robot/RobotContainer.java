/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.DriveCommand;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.commands.SpinUpCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.teleop.OI;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final DriveCommand m_driveCommand = new DriveCommand(m_drivetrainSubsystem);
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final Shooter m_shooter = new Shooter();

  //private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  //private final DriveCommand m_driveCommand = new DriveCommand(m_drivetrainSubsystem);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSubsystem, m_driveCommand);
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("---------------inside configureButtonBindings()");

    JoystickButton limelightAlignButton = new JoystickButton(OI.getXboxController(), 2);
    limelightAlignButton.whileHeld(new LimelightAlignCommand(m_limelightSubsystem, m_drivetrainSubsystem));

    // JoystickButton spinUpShooterButton = new
    // JoystickButton(OI.getXboxController(), 4);
    // spinUpShooterButton.whileHeld(new SpinUpCommand(m_ShooterSubsystem));

    JoystickButton spinUpShooterButton = new JoystickButton(OI.getXboxController(), 4);
    spinUpShooterButton.whileHeld(new SpinUpCommand(m_shooter));

    JoystickButton parallelCommandButton = new JoystickButton(OI.getXboxController(), 3);
    parallelCommandButton.whileHeld(new AlignAndShootCommand(m_limelightSubsystem, m_drivetrainSubsystem, m_shooter));
    System.out.println("button 3 just pressed--------------------------");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
