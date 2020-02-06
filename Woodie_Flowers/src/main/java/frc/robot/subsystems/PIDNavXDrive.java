/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.Utilities;
import frc.robot.teleop.OI;
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
  Utilities misterG = new Utilities();
  WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
  WPI_TalonSRX leftBack = new WPI_TalonSRX (2);
  WPI_TalonSRX rightFront = new WPI_TalonSRX(5);
  WPI_TalonSRX rightBack = new WPI_TalonSRX(4);
  OI axis = new OI();
  boolean theNextLevel = false;
  public boolean interupted =  false;
  double previousAngle = 0;
  int previousDistance = 0;
  int ammmountOfTimes = 0;
  public boolean gogoGadget = false;
  public boolean wakawaka = false;
  public long time = 0;
  SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightFront, rightBack);
  private DifferentialDrive robotDrive = new DifferentialDrive(leftGroup, rightGroup);
  boolean driveIsActive;
  public PIDNavXDrive() {
    // Intert a subsystem name and PID values here
    super(new PIDController(0,0,0));
    getController().setPID(misterG.k_PTurn, misterG.k_ITurn, misterG.k_DTurn);
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
    robotDrive.arcadeDrive(speed , turn);
  }
  public boolean getDriveActive(){
    return driveIsActive;
  }
  public void setDriveActive(boolean statement){
    driveIsActive = statement;
  }
  public boolean getInterupted(){
    return interupted;
  }
  public void setInterupted(boolean statement){
    interupted= statement;
  }
  public void resetDistance(){
    previousDistance = 0;
  }
  public int getPreviousDistance() {
    return previousDistance;
  }
  public void setLevel(boolean statement){
    theNextLevel= statement;
  }
  public boolean getLevel() {
    return theNextLevel;
  }
  public void setResetTime(int time){
    ammmountOfTimes = time;
  }
  public int getResetTimes() {
    return ammmountOfTimes;
  }
  public void setPreviousDistance(int distance){
    previousDistance = distance;
  }
  public double getPreviousAngle() {
    return previousAngle;
  }
  public void setPreviousAngle(double angle){
    previousAngle = angle;
  }
  @Override
  protected void useOutput(double output, double setpoint) {
    robotDrive.arcadeDrive(0 , output);

  }
  public int getEncoderLeft(){
    return leftFront.getSelectedSensorPosition();
  }
  public void resetEncoderValues(){
    leftFront.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
  }
  public int getEncoderRight(){
    return rightBack.getSelectedSensorPosition();
  }
  public void drive(){
    robotDrive.arcadeDrive(OI.getXboxController().getRawAxis(1), OI.getXboxController().getRawAxis(4));
  }
  @Override
  public double getMeasurement() {
    //System.out.println("getMeasurement is working, navx angle is: " + navX.getAngle()+ ", Position error == " + getController().getPositionError());
    getController().getPositionError();
    return navX.getAngle();
    
  }
  public boolean atSetPoint(){
    return misterG.atSetPoint;
  }
  @Override
  public void enable() {
    System.out.println("enable");
    leftFront.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
    navX.reset();
    super.enable();
  }
}