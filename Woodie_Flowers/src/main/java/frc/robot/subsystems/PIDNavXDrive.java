/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import frc.robot.subsystems.Constants;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
/**
 * Add your docs here.
 */
public class PIDNavXDrive extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  AHRS navX = new AHRS(SerialPort.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
  Constants misterG = new Constants();
  CANSparkMax leftFront = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(3, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive robotDrive = new DifferentialDrive(leftGroup, rightGroup);
  public PIDNavXDrive() {
    // Intert a subsystem name and PID values here
    super(new PIDController(0,0,0));
    getController().setPID(misterG.k_P, misterG.k_I, misterG.k_D);
    getController().enableContinuousInput(-180, 180);
    //getController().setTolerance(5);
    robotDrive.setSafetyEnabled(false);
    //LiveWindow.enableTelemetry(getController());
  }
  
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  

  public void robotDrive (double speed, double turn){
    System.out.println("robotDrive");
    robotDrive.arcadeDrive(speed , turn);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    System.out.println("useOutput " + output + ", Setpoint: "+ setpoint);
    robotDrive.arcadeDrive(0 , output);

  }

  @Override
  public double getMeasurement() {
    System.out.println("getMeasurement is working, navx angle is: " + navX.getAngle()+ ", Position error == " + getController().getPositionError());
    getController().getPositionError();
    return navX.getAngle();
    
  }
  public boolean atSetPoint(){
    System.out.println("atSetPoint "+ getController().atSetpoint());
    
    return misterG.atSetPoint;
  }
  @Override
  public void enable() {
    System.out.println("enable");
    navX.reset();
    super.enable();
  }
}