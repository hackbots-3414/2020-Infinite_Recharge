/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController pad = new XboxController(0);
  CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(31, MotorType.kBrushless);
  CANSparkMax topBelt = new CANSparkMax(42, MotorType.kBrushless);
  CANSparkMax midBelt = new CANSparkMax(41, MotorType.kBrushless);
  CANSparkMax lowBelt = new CANSparkMax(43, MotorType.kBrushless);

  TalonFX shooterA = new TalonFX(11);
  TalonFX shooterB = new TalonFX(12);
  TalonSRX climberA = new TalonSRX(21);
  TalonSRX climberB = new TalonSRX(22);
  SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive drivetrain = new DifferentialDrive(left, right);
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();
    leftFront.getEncoder().setPosition(0);
    leftBack.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);
    rightBack.getEncoder().setPosition(0);

    leftFront.setInverted(true);
    leftBack.setInverted(true);
   
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  public void printEncoders(){
    System.out.println(" LeftFront Position: " + leftFront.getEncoder().getPosition());
    System.out.println(" LeftBack  Position: " + leftBack.getEncoder().getPosition());
    System.out.println("RightFront Position: " + rightFront.getEncoder().getPosition());
    System.out.println(" RightBack Position: " + rightBack.getEncoder().getPosition());
    
    //Timer.delay(1.0);

  }
  public void printVelocity(){
    System.out.println(" LeftFront Position: " + leftFront.getEncoder().getVelocity());
    System.out.println(" LeftBack  Position: " + leftBack.getEncoder().getVelocity());
    System.out.println("RightFront Position: " + rightFront.getEncoder().getVelocity());
    System.out.println(" RightBack Position: " + rightBack.getEncoder().getVelocity());
    
  }
  @Override
  public void teleopPeriodic() {
  drivetrain.arcadeDrive((pad.getX(Hand.kRight)),(-pad.getY(Hand.kLeft)));
    if(pad.getAButton()){
      shooterA.set(ControlMode.PercentOutput, -0.2);
      shooterB.set(ControlMode.PercentOutput, 0.2);
    }
    else{
      shooterA.set(ControlMode.PercentOutput, 0);
      shooterB.set(ControlMode.PercentOutput, 0);
    }
    if(pad.getBButton()){
      climberA.set(ControlMode.PercentOutput, 0.2);
      climberB.set(ControlMode.PercentOutput, -0.2);
    }
    else{
      climberA.set(ControlMode.PercentOutput, 0);
      climberB.set(ControlMode.PercentOutput, 0);
    }
    if(pad.getYButton()){
      intake.set(0.25);
    } else{
      intake.set(0);
    }
    if (pad.getXButton()){
      lowBelt.set(-0.15);
      midBelt.set(0.25);
      topBelt.set(0.15);
    } else{
      lowBelt.set(0);
      midBelt.set(0);
      topBelt.set(0);
    }
    /* if(pad.getXButton()){
       leftFront.set(0.2);
       leftBack.set(0.2);
       rightFront.set(0.2);
       rightBack.set(0.2);
     }
     else{
       leftFront.set(0);
       leftBack.set(0);
       rightFront.set(0);
       rightBack.set(0);
     }*/
    // printEncoders();
     printVelocity();


  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
