/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
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
  double encoderConstant = (1 / 8.68) * 0.155 * Math.PI;
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  //private final SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  //private final SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive m_drivetrain = new DifferentialDrive(leftBack, rightFront);
  private final AHRS NavX = new AHRS(Port.kMXP);
  DifferentialDriveOdometry m_odometry;


  public DrivetrainSubsystem() {
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
    leftBack.getEncoder().setVelocityConversionFactor(encoderConstant/60);
    rightFront.getEncoder().setVelocityConversionFactor(encoderConstant/60);
   // rightFront.getEncoder().setInverted(true);
    leftFront.follow(leftBack);
    rightBack.follow(rightFront);



   m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  
} 

  public double getHeading() {
    return Math.IEEEremainder(NavX.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public double getLeftDistance(){
    //System.out.println("left distance: " + leftBack.getEncoder().getPosition());
    return leftBack.getEncoder().getPosition();
  }
  public double getRightDistance(){
    //System.out.println("rightt distance: " + rightFront.getEncoder().getPosition());
    return rightFront.getEncoder().getPosition();
  }
//  public double getAverageDistance(){
//    return (getLeftDistance() +  getRightDistance()) / 2;
//  }
  public double getLeftVelocity(){
   // System.out.println("left velocity: " + leftBack.getEncoder().getVelocity());
    return leftBack.getEncoder().getVelocity();
  }
  public double getRightVelocity(){
   // System.out.println("right velocity: " + rightFront.getEncoder().getVelocity());
    return rightFront.getEncoder().getVelocity();
  }
  
  public void printEncoderValues(){
   /* System.out.println("//////////////////////// Left Encoder Posision: " + getLeftDistance());
    System.out.println("//////////////////////// Left Encoder Velocity: " + getLeftVelocity());
    System.out.println("/////////////////////// Right Encoder Posision: " + getRightDistance());
    System.out.println("/////////////////////// Right Encoder Velocity: " + getRightVelocity());
*/
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(),
    getRightDistance());
    
    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }
  
  public void drive(){
    m_drivetrain.arcadeDrive(-OI.getJoystick().getRawAxis(1), OI.getJoystick().getRawAxis(2));
    //printEncoderValues();
  }
  public void setMaxOutput(double maxOutput){
    m_drivetrain.setMaxOutput(maxOutput);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
   // System.out.println("Angle: " + NavX.getAngle());
    //System.out.println("left volts: " + leftVolts + " right volts: " + rightVolts);
   // printEncoderValues();
    leftBack.setVoltage(leftVolts);
    rightFront.setVoltage(-rightVolts);
    m_drivetrain.feed();
  }
  public double getTurnRate() {
    return NavX.getRate() * (false ? -1.0 : 1.0);
  }

public void resetEncoders() {
  leftBack.getEncoder().setPosition(0);
  rightFront.getEncoder().setPosition(0);
}

}