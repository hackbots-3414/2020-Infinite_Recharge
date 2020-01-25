/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDNavXDrive;
import frc.robot.subsystems.Constants;


public class TurnDotEXE extends CommandBase {
  PIDNavXDrive navXDrive = null; 
  double vibeCheck;
  Constants coolCat = new Constants();
  double tolerbo;
  public TurnDotEXE (final PIDNavXDrive pidNavXDrive,double angularBruhMoment,double tolerboi) {
    addRequirements(pidNavXDrive);
    navXDrive = pidNavXDrive;
    vibeCheck = angularBruhMoment;
    tolerbo = tolerboi;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("initialize");
    navXDrive.enable();
    navXDrive.setSetpoint(vibeCheck);
    coolCat.setAtSetpoint(false);
 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    System.out.println("execute");
    navXDrive.setSetpoint(vibeCheck);
    System.out.println("setSetpoint "+ navXDrive.getController().getSetpoint());
    navXDrive.getMeasurement();
    if (vibeCheck<0){
      coolCat.toler = -tolerbo;
      System.out.println("tolerance; " + tolerbo);
      System.out.println("tolerance; " + coolCat.toler);
    }

    if(vibeCheck>0){
      coolCat.toler = tolerbo;
    }
    

       
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    System.out.println("isFinished: " + navXDrive.atSetPoint());
    System.out.println("tolerance: "+ coolCat.toler);
    System.out.println("tolerance: "+ tolerbo);
      if( navXDrive.getController().getPositionError() > coolCat.toler && coolCat.toler<0)  {
      navXDrive.disable();
      coolCat.setAtSetpoint(true);
      System.out.println("isFinished: " + true);
      System.out.println("Misa finished");
      return coolCat.atSetPoint;
      }
    if(navXDrive.getController().getPositionError() < coolCat.toler && coolCat.toler > 0)  {
      navXDrive.disable();
      coolCat.setAtSetpoint(true);
      System.out.println("isFinished: " + true);
      System.out.println("Misa finished");
      return coolCat.atSetPoint;
    }
    

    return coolCat.atSetPoint;
   
  }

  // Called once after isFinished returns true


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

}
