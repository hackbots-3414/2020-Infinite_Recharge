/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.teleop.OI;

public class DrivetrainSubsystem extends SubsystemBase {  /**
   *
   * Creates a new DrivetrainSubsystem.
   */
  /*CANSparkMax leftFront = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(5, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);*/
  /*WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
  WPI_TalonSRX leftBack = new WPI_TalonSRX (2);
  WPI_TalonSRX rightFront = new WPI_TalonSRX(5);
  WPI_TalonSRX rightBack = new WPI_TalonSRX(4);*/
  Talon leftFront = new Talon(4);
  Talon leftBack = new Talon (5);
  Talon rightFront = new Talon(2);
  Talon rightBack = new Talon(3);
  SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  public DifferentialDrive m_drivetrain = new DifferentialDrive(left, right);
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //left.setInverted(true);
  }

  public void drive(){
    m_drivetrain.arcadeDrive(-OI.getJoystick().getRawAxis(1), OI.getJoystick().getRawAxis(0));

  }
  public void drive(double speed , double rotation){
    m_drivetrain.arcadeDrive(speed, rotation);

  }
}