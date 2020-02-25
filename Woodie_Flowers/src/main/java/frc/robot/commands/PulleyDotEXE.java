/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.teleop.OI;

public class PulleyDotEXE extends CommandBase {
  /**
   * Creates a new PulleyDotEXE.
   */
  PulleySubsystem pully = new PulleySubsystem();
  public PulleyDotEXE() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pully.setEncoder(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pully.setPullEy(OI.getXboxController().getRawAxis(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pully.getEncoder()> 1000000000){
      pully.setPullEy(0);
      return true;
    }
    return false;
  }
}
