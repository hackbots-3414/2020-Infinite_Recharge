/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;
import frc.robot.teleop.OI;

public class DrivetrainSubsystem extends SubsystemBase {  /**
   *
   * Creates a new DrivetrainSubsystem.
   */
  
  CANSparkMax leftFront = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(3, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  private final SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  private final SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive m_drivetrain = new DifferentialDrive(left, right);
  private final AHRS NavX = new AHRS();
  DifferentialDriveOdometry m_odometry;
  double encoderConstant = (1 / (4096/4)) * 0.15 * Math.PI * 116.16;
  public DrivetrainSubsystem() {
    // Sets the distance per pulse for the encoders
    leftFront.getEncoder().setPositionConversionFactor(encoderConstant);
    rightFront.getEncoder().setPositionConversionFactor(encoderConstant);

   m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  
} 

  public double getHeading() {
    return Math.IEEEremainder(NavX.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public double getLeftDistance(){
    return leftFront.getEncoder().getPosition();
  }
  public double getRightDistance(){
    return rightFront.getEncoder().getPosition();
  }
  public double getAverageDistance(){
    return (leftFront.getEncoder().getPosition() + rightFront.getEncoder().getPosition()) / 2;
  }
  public double getLeftVelocity(){
    return leftFront.getEncoder().getVelocity();
  }
  public double getRightVelocity(){
    return rightFront.getEncoder().getVelocity();
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(),
    getRightDistance());
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }
  
  public void drive(){
    m_drivetrain.arcadeDrive(-OI.getJoystick().getRawAxis(1), OI.getJoystick().getRawAxis(2));

  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    System.out.println("left volts: " + leftVolts + " right volts: " + rightVolts);
    left.setVoltage(leftVolts);
    right.setVoltage(-rightVolts);
    m_drivetrain.feed();
  }
  public double getTurnRate() {
    return NavX.getRate() * (false ? -1.0 : 1.0);
  }
}