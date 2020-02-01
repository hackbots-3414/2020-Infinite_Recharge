/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDNavXDrive;
import frc.robot.subsystems.Utilities;;


public class TurnDotEXE extends CommandBase {
  PIDNavXDrive navXDrive = null; 
  double m_angle;
  public Utilities util = new Utilities();
  public double m_tolerance;
  double initialRefrenceAngle;
  public TurnDotEXE (final PIDNavXDrive pidNavXDrive,double angularBruhMoment,double m_tolerancei) {
    addRequirements(pidNavXDrive);
    navXDrive = pidNavXDrive;
    m_angle = angularBruhMoment;
    m_tolerance = m_tolerancei;
    initialRefrenceAngle = m_angle;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("initialize");
    navXDrive.enable();
    navXDrive.setSetpoint(m_angle);
    util.setAtSetpoint(false);
 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //System.out.println("execute");
    navXDrive.setSetpoint(m_angle);
    //System.out.println("setSetpoint "+ navXDrive.getController().getSetpoint());
    navXDrive.getMeasurement();
    if (m_angle<0){
      util.toler = -m_tolerance;
      //System.out.println("tolerance; " + m_tolerance);
      //System.out.println("tolerance; " + util.toler);
    }

    else{
      util.toler = m_tolerance;
    }  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
   // System.out.println("isFinished: " + navXDrive.atSetPoint());
    //System.out.println("tolerance: "+ util.toler);
    //System.out.println("tolerance: "+ m_tolerance);
      if( navXDrive.getController().getPositionError() > util.toler && util.toler<0)  {
      navXDrive.disable();
      util.setAtSetpoint(true);
     // System.out.println("isFinished: " + true);
      //System.out.println("Misa finished");
      return util.atSetPoint;
      }
    if(navXDrive.getController().getPositionError() < util.toler && util.toler > 0)  {
      navXDrive.disable();
      util.setAtSetpoint(true);
      //System.out.println("isFinished: " + true);
      //System.out.println("Misa finished");
      return util.atSetPoint;
    }
    else{
      return util.atSetPoint;
    }
    
   
  }

  // Called once after isFinished returns true


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

}
