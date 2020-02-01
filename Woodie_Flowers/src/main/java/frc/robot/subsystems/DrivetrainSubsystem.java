/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.teleop.OI;

public class DrivetrainSubsystem extends SubsystemBase {  /**
   *
   * Creates a new DrivetrainSubsystem.
   */
  /*WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
  WPI_TalonSRX leftBack = new WPI_TalonSRX(2);
  WPI_TalonSRX rightFront = new WPI_TalonSRX(3);
  WPI_TalonSRX rightBack = new WPI_TalonSRX(4);
  SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive m_drivetrain = new DifferentialDrive(left, right);*/
  
  //make 3 new classes getleftencoder getrightencoder and getaverage encoder

  public int getLeftenCoder(){
    //return leftFront.get
    return 1;
  } 


  // Odometry class for tracking robot pose
  //private final DifferentialDriveOdometry m_odometry;
  //private final Gyro m_gyro = new ADXRS450_Gyro();

  private final Encoder m_leftEncoder = new Encoder(1, 2);
  private final Encoder m_rightEncoder = new Encoder(3, 4);

  
  
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