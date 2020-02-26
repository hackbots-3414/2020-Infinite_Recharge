/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeltSubsyteem;

public class BeltShootCommand extends CommandBase {

  BeltSubsyteem theBeltBois;
  boolean running;
  //back 1
  //front 0
  public BeltShootCommand(BeltSubsyteem belt) {
    theBeltBois = belt;
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
    running = true;
    theBeltBois.beltMethod(0.5);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return !running;
  }
  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    running = false;
    theBeltBois.beltMethod(0.0);
    super.end(interrupted);
  }

  // Returns true when the command should end.

  
}
