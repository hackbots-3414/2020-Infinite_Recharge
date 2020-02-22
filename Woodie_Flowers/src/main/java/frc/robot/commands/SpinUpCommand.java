package frc.robot.commands;
/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpinUpCommand extends CommandBase {
  static {
    SmartDashboard.putNumber("shooterVelocity", 0);
    // System.out.println("shooter velocity: " +
    // SmartDashboard.getData("shooterVelocity"));
  }
  private final Shooter shooterSubsystem;
  double velocity;

  public SpinUpCommand(final Shooter shooterSubsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called just before this Command runs the first time

  public void initialize() {
    // shooterSubsystem.setSetpoint(SmartDashboard.getNumber("shooterVelocity", 0));
    // TODO any output needed for SmartDashboard
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    super.execute();
    shooterSubsystem.shoot();

    /*
     * shooterSubsystem.enable();
     * System.out.println("inside SpinUpCommand execute()");
     * SmartDashboard.putNumber("shooterVelocity", 0);
     */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    /*
     * if(shooterSubsystem.isReadyToShoot()) { return true; }
     */
    return false;
    /*
     * shooterSubsystem.atSetpoint(); System.out.println("current velocity: " +
     * shooterSubsystem.getAverageShooterVelocity());
     * System.out.println("current setpoint: " + shooterSubsystem.getSetpoint());
     * if(shooterSubsystem.atSetpoint() == true){ System.out.println("at setpoint");
     * Timer.delay(10); return true; } return false;
     */
    // isReadyToShoot();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  public void interrupted() {
    end(false);
  }
}