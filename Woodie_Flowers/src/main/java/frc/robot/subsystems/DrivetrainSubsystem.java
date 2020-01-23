/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.teleop.OI;
import edu.wpi.first.networktables.*;

public class DrivetrainSubsystem extends SubsystemBase {  /**
   *
   * Creates a new DrivetrainSubsystem.
   */
  CANSparkMax leftFront = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(3, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive m_drivetrain = new DifferentialDrive(left, right);
  
  double left_Command;
  double right_Command;
  

  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(){
    m_drivetrain.arcadeDrive(-OI.getJoystick().getRawAxis(1), OI.getJoystick().getRawAxis(2));
  }

  public void steering() {
  
    float kP = -0.1f;
    float min_command = -0.05f;
    NetworkTable table = new NetworkTable.getTable("limelight");
    float tx = table.getNumber("tx");

    if(m_Joystick.getRawButton(2)) {

      float heading_error = -tx;
      float steering_adjust = 0.0f;

      if(tx > 1.0) {
          steering_adjust = Kp * heading_error - min_command;
      } else if(tx < 1.0) {
          steering_adjust = Kp * heading_error + min_command;
      }

      left_command += steering_adjust; 
      right_command -= steering_adjust;

    }
  }

  public void estimateDistance() {

  }

  public void getDistance() {
    float kPDistance =  -0.1f;
    float currentDistance = estimateDistance();

    if(m_Joystick.getRawButton(3)){
      float distanceError = desiredDistance - currentDistance;
      drivingAdjust = kPDistance * distanceError;

      left_command += distanceAdjust;
      right_command += distanceAdjust;
    }
  }
  
}