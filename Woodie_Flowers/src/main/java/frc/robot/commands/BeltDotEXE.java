/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.ietf.jgss.Oid;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.teleop.OI;

public class BeltDotEXE extends CommandBase {

  BeltSubsyteem theBeltBois;
  //back 1
  //front 0
  double output;
  public BeltDotEXE(BeltSubsyteem belt,double speed) {
    theBeltBois = belt;
    output = speed;
    addRequirements(belt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean check = true;
   // SmartDashboard.putBoolean("Back", theBeltBois.irsback.get());
    //System.out.println("Front irs: "+ theBeltBois.irsfront.get());
    //System.out.println("Back irs: "+ theBeltBois.irsback.get());
    if(!theBeltBois.irsback.get() && theBeltBois.irsfront.get()){
      SmartDashboard.putBoolean("irsBack", theBeltBois.irsback.get());
      theBeltBois.beltMethod(output);
    }
    else{
      theBeltBois.beltMethod(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.

  
}
