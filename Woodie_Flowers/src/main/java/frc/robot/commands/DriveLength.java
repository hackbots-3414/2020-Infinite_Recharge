/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveLength extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final double lengthDriven;

  /**
   * Creates a new driveLegnth.
   */
  public DriveLength(DrivetrainSubsystem driveTrain, double lengthDriven) {
    // Use addRequirements() here to declare subsystem dependencies.
    super();
    this.drivetrainSubsystem = driveTrain;
    this.lengthDriven = lengthDriven;

    addRequirements(drivetrainSubsystem);
  } 
  } 

  // Called when the command is initially scheduled.
  @Override
  
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
