/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;
import frc.robot.teleop.OI;
import edu.wpi.first.wpilibj.SerialPort;

public class DrivetrainSubsystem extends PIDSubsystem {
  /**
   *
   * Creates a new DrivetrainSubsystem.
   */
  AHRS navX = new AHRS(SerialPort.Port.kMXP);
  CANSparkMax leftFront = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(5, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive m_drivetrain = new DifferentialDrive(left, right);
  public boolean interupted = false;

  boolean driveIsActive;
  double previousAngle = 0;
  double previousDistance = 0;

  DifferentialDriveOdometry m_odometry;
  double encoderConstant = (1 / (42)) * 0.15 * Math.PI * 5.1742031134;

  public DrivetrainSubsystem() {
    super(new PIDController(0, 0, 0));

    getController().setPID(Utilities.k_PTurn, Utilities.k_ITurn, Utilities.k_DTurn);
    getController().enableContinuousInput(-180, 180);

    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();
    leftFront.getEncoder().setPosition(0);
    leftBack.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);
    rightBack.getEncoder().setPosition(0);
    // Sets the distance per pulse for the encoders
    leftBack.getEncoder().setPositionConversionFactor(encoderConstant);
    rightFront.getEncoder().setPositionConversionFactor(encoderConstant);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }

  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getLeftDistance() {
    return -leftBack.getEncoder().getPosition();
  }

  public double getRightDistance() {
    return -rightFront.getEncoder().getPosition();
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public double getLeftVelocity() {
    return leftBack.getEncoder().getVelocity();
  }

  public double getRightVelocity() {
    return -rightFront.getEncoder().getVelocity();
  }

  public void printEncoderValues() {
    System.out.println("//////////////////////// Left Encoder Posision: " + getLeftDistance());
    System.out.println("//////////////////////// Left Encoder Velocity: " + getLeftVelocity());
    System.out.println("/////////////////////// Right Encoder Posision: " + getRightDistance());
    System.out.println("/////////////////////// Right Encoder Velocity: " + getRightVelocity());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
  }

  public void drive() {
    m_drivetrain.tankDrive(-OI.getRightJoystick().getY(), OI.getLeftJoystick().getY());
  }

  public void drive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public void stop() {
    left.set(0.0);
    right.set(0.0);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    System.out.println("left volts: " + leftVolts + " right volts: " + rightVolts);
    printEncoderValues();
    left.setVoltage(leftVolts);
    right.setVoltage(-rightVolts);
    m_drivetrain.feed();
  }

  public double getTurnRate() {
    return navX.getRate() * (false ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    leftBack.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);
  }

  // start PID navX code
  public boolean getDriveActive() {
    return driveIsActive;
  }

  public void setDriveActive(boolean statement) {
    driveIsActive = statement;
  }

  public boolean getInterupted() {
    return interupted;
  }

  public void setInterupted(boolean statement) {
    interupted = statement;
  }

  public void resetDistance() {
    previousDistance = 0;
  }

  public double getPreviousDistance() {
    return previousDistance;
  }

  public void setPreviousDistance(double distance) {
    previousDistance = distance;
  }

  public double getPreviousAngle() {
    return previousAngle;
  }

  public void setPreviousAngle(double angle) {
    previousAngle = angle;
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    m_drivetrain.arcadeDrive(0, output);

  }

  public void setPIDValues(double P, double I, double D) {
    getController().setP(P);
    getController().setI(I);
    getController().setD(D);
  }

  @Override
  public double getMeasurement() {
    // System.out.println("getMeasurement is working, navx angle is: " +
    // navX.getAngle()+ ", Position error == " +
    // getController().getPositionError());
    getController().getPositionError();
    return navX.getAngle();

  }

  public boolean atSetPoint() {
    return Utilities.atSetPoint;
  }

  @Override
  public void enable() {
    System.out.println("enable");
    resetEncoders();
    navX.reset();
    super.enable();
  }
}