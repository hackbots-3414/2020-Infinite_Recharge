/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class Constants extends SubsystemBase {
  /**
   * Add your docs here.
   */
  public Constants() {
    // Intert a subsystem name and PID values here
    super();
    
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }
  double k_P = 0.015;
  double k_I = 0.00361446;
  double k_D = 0.0155625;
  public boolean atSetPoint = false;
  public double toler;
  //0.015, 0.00361446, 0.0155625


  public void setAtSetpoint(boolean value){
    atSetPoint = value;
  }
}
