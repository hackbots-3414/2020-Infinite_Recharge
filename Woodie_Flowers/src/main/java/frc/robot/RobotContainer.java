/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.BeltDotEXE;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveDotEXE;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.commands.ShootSequenceCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.TurnDotEXE;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.teleop.OI;

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
  boolean counterV2;
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final DriveCommand m_driveCommand = new DriveCommand(m_drivetrainSubsystem);
  private final Shooter m_shooter = new Shooter();
  TurnDotEXE stay0degrees = new TurnDotEXE(m_drivetrainSubsystem, 5, 1);
  DriveDotEXE forward = new DriveDotEXE(200000, 0.5, 6,m_drivetrainSubsystem);
  private final StopCommand m_stop = new StopCommand(m_shooter, m_drivetrainSubsystem);

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final BeltSubsyteem m_belt = new BeltSubsyteem();
  BeltDotEXE beltCommand = new BeltDotEXE(m_belt, m_intake);

  // private final DrivetrainSubsystem m_drivetrainSubsystem = new
  // DrivetrainSubsystem();
  // private final DriveCommand m_driveCommand = new
  // DriveCommand(m_drivetrainSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSubsystem, m_driveCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_belt,beltCommand);
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

    bindCommandToButton(new LimelightAlignCommand(m_limelightSubsystem, m_drivetrainSubsystem), 1);
    //bindCommandToButton(new SpinUpCommand(m_shooter), 2);
    bindCommandToButton(new AlignAndShootCommand(m_limelightSubsystem, m_drivetrainSubsystem, m_shooter), 3);
    bindCommandToButton(new ShootSequenceCommand(m_belt, m_drivetrainSubsystem, m_shooter), 4);
    bindCommandToButton(new IntakeCommand(m_intake), 5);

  }

  public void bindCommandToButton(final Command command, final int buttonNumber) {
    final JoystickButton joystickButton = new JoystickButton(OI.getOperatorPad(), buttonNumber);
    joystickButton.whileHeld(command);
  }

  public void whenPressedButton(final Command command, final int buttonNumber) {
    final JoystickButton joystickButton = new JoystickButton(OI.getOperatorPad(), buttonNumber);
    joystickButton.whenPressed(command);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
