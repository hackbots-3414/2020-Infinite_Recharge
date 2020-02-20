/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.subsystems.IRSensor;
import frc.robot.teleop.OI;

public class BeltDotEXE extends CommandBase {
  BeltSubsyteem theBeltBois = new BeltSubsyteem();
  IRSensor irsfront = new IRSensor(0);
  IRSensor irsback = new IRSensor(1);
  //back 1
  //front 0
  public BeltDotEXE() {

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    theBeltBois.beltMethod(0.15);
    System.out.println("Front irs: "+ irsfront.getStatus());
    System.out.println("Back irs: "+ irsback.getStatus());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(OI.getXboxController().getAButton() == false){
    theBeltBois.beltMethod(0.0);
    System.out.println("Belt Is Done");
    return true;
    }
    else{
      return false;
    }
  }

  
}
