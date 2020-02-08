package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.All_In_One;
import frc.robot.commands.Backwards;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Forward;
import frc.robot.commands.All_In_One;
import frc.robot.commands.Backwards;
import frc.robot.commands.Forward;
import frc.robot.commands.Right_Turn;
import frc.robot.commands.Left_Turn;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.teleop.OI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DriveCommand m_driveCommand = new DriveCommand(m_drivetrainSubsystem);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSubsystem, m_driveCommand);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
    JoystickButton Button_Turn_Right = new JoystickButton(OI.getJoystick(),1);
    Button_Turn_Right.whenPressed(new Right_Turn(m_drivetrainSubsystem));
    System.out.println("Turning Right for 3 seconds(3000 milliseconds....................) ");

    JoystickButton toggleButton = new JoystickButton(OI.getJoystick(),2);
    toggleButton.whenPressed(new Forward(m_drivetrainSubsystem));
    System.out.println("Going Forward for 3 seconds(3000 milliseconds)...................");

   JoystickButton toggleButton3 = new JoystickButton(OI.getJoystick(),3);
    toggleButton3.whenPressed(new All_In_One(m_drivetrainSubsystem));
    System.out.println("Going forward for 3 seconds(3000 milliseconds), then turning Right for 3 seconds, then turning Left for 1 seconds(1000 milliseconds), then going Backwards for 3 seconds(3000 milliseconds)..............");

    JoystickButton toggleButton4 = new JoystickButton(OI.getJoystick(),4);
    toggleButton4.whenPressed(new Backwards(m_drivetrainSubsystem));
    System.out.println("Going Backwards for 3 seconds(3000 milliseconds)................");
    
    JoystickButton toggleButton5 = new JoystickButton(OI.getJoystick(),5);
    toggleButton5.whenPressed(new Left_Turn(m_drivetrainSubsystem));
    System.out.println("Turning Left for 1 second(1000 milliseconds......................");
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}