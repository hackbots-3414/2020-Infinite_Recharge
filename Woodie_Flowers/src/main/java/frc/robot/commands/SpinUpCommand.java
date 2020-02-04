package frc.robot.commands;
/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpinUpCommand extends CommandBase{
static{
  SmartDashboard.putNumber("shooterVelocity", 0);
}
  private final ShooterSubsystem shooterSubsystem;
  double velocity;
  public SpinUpCommand(final ShooterSubsystem shooterSubsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called just before this Command runs the first time

  public void initialize() {
    shooterSubsystem.setSetpoint(SmartDashboard.getNumber("shooterVelocity", 0));
  }


  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    super.execute();
    shooterSubsystem.enable();
    System.out.println("inside SpinUpCommand execute()");
  }

  // Make this return true when this Command no longer needs to run execute()
  
  public boolean isFinished() {
     shooterSubsystem.atSetpoint();
     System.out.println("current velocity: " + shooterSubsystem.getAverageShooterVelocity());
     System.out.println("current setpoint: " + shooterSubsystem.getSetpoint());
     if(shooterSubsystem.atSetpoint() == true){
       System.out.println("at setpoint");
       Timer.delay(10);
       return true;
     } 
     return false;
  }

  // Called once after isFinished returns true
  public void end() {
    shooterSubsystem.disable();
    shooterSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  public void interrupted() {
    end();
  }
}
