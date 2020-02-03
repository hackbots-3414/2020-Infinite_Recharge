/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

//import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {  /**
   *
   * Creates a new DrivetrainSubsystem.
   */
  CANSparkMax leftFront = new CANSparkMax(10, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(20, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(30, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(40, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive m_drivetrain = new DifferentialDrive(left, right);
  
  //make 3 new classes getleftencoder getrightencoder and getaverage encoder

  public int getLeftenCoder(){
    //return leftFront.get
    return 1;
  } 


  // Odometry class for tracking robot pose
  //private final DifferentialDriveOdometry m_odometry;
  //private final Gyro m_gyro = new ADXRS450_Gyro();

  private final Encoder m_leftEncoder = new Encoder(10, 20);
  private final Encoder m_rightEncoder = new Encoder(30, 40);
  /*public void DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
  
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  
  */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }
  /*public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  */
  public void drive(){
    //m_drivetrain.arcadeDrive(-OI.getXboxController().getY(Hand.kLeft), OI.getXboxController().getX(Hand.kRight));

  }
}