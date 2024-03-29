/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  IntakeSubsystem intake;
  BeltSubsyteem belt;
  public IntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    intake.goDown();
    intake.setSpeed(0.35);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.goUp();
  }

}
