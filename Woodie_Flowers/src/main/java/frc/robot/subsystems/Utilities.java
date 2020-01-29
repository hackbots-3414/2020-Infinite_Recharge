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
public class Utilities extends SubsystemBase {
  /**
   * Add your docs here.
   */
  public Utilities() {
    // Intert a subsystem name and PID values here
    super();
    
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }
  public double k_PTurn = 0.015;
  public double k_ITurn = 0.00361446;
  public double k_DTurn = 0.0155625;
  public double k_PDrive = 0.015;
  public double k_IDrive = 0.00361446;
  public double k_DDrive = 0.0155625;
  public boolean atSetPoint = false;
  public double toler;
  //0.015, 0.00361446, 0.0155625


  public void setAtSetpoint(boolean value){
    atSetPoint = value;
  }
}
