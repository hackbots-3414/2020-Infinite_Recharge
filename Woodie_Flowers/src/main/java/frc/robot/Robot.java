/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.teleop.OI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.Drive;
import frc.robot.commands.DriveDotEXE;
import frc.robot.commands.TurnDotEXE;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Utilities;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public DrivetrainSubsystem pidNavX = new DrivetrainSubsystem();
  Utilities values = new Utilities();
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected;
  boolean counterV2;
  TurnDotEXE stay0degrees = new TurnDotEXE(pidNavX, 5, 1);
  DriveDotEXE forward = new DriveDotEXE(200000, 0.5, 6, pidNavX);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CommandScheduler.getInstance().registerSubsystem(pidNavX);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  boolean counter;

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    /*m_autoSelected = m_chooser.getSelected();
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
    pidNavX.resetDistance();
    pidNavX.enable();
    if (counter == true) {
      counter = false;
      // CommandScheduler.getInstance().schedule(new TurnDotEXE(pidNavX,-90,5));
      CommandScheduler.getInstance().schedule(new Drive(20000000, 0.5, pidNavX, 5));
      System.out.println("worked");

    }
    counter = true;*/
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

/*@Override
  public void autonomousPeriodic() {
    boolean stooop = false;

    if (counter == true) {
      counter = false;
      // CommandScheduler.getInstance().schedule(new TurnDotEXE(pidNavX,180,3));
      // CommandScheduler.getInstance().schedule(new DriveDotEXE(20000,0.5));
      pidNavX.enable();
      // CommandScheduler.getInstance().schedule(forward);
      System.out.println("worked");
      pidNavX.setPreviousDistance(0);
      CommandScheduler.getInstance().schedule(new TurnDotEXE(pidNavX, -90, 5));
      counterV2 = false;

    }
    if (forward.isFinished() == true && pidNavX.getInterupted() == true && counter == false && counterV2 == true) {
      CommandScheduler.getInstance().cancel(forward);
      CommandScheduler.getInstance().schedule(stay0degrees);
      System.out.println("pidNavx schedule");
      pidNavX.setInterupted(false);
      counterV2 = false;
    }
    if (counterV2 == false && counter == false && Math.abs(pidNavX.getMeasurement()) > 85 && stooop == false) {
      pidNavX.enable();
      CommandScheduler.getInstance().schedule(forward);
      System.out.println("drive scheduled");
      counterV2 = true;
      stooop = true;
    }
    if (stay0degrees.isFinished() == true && counterV2 == false && counter == false) {
      CommandScheduler.getInstance().cancel(stay0degrees);
      CommandScheduler.getInstance().schedule(forward);
      System.out.println("drive scheduled");
      counterV2 = true;
    }

  }*/