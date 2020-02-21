
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class Backwards extends CommandBase {
  public static final String RobotMap = null;
  public static final long DURRATION_IN_MILLISECONDS = 3000;
  public static final double SPEED = 0;
  public static final double ROTATION = -0.5;
  public static final double seconds = 5;
  
  private final DrivetrainSubsystem drivetrainSubsystem;
  private long startTime = 0;
  private boolean running = false;
  DigitalInput limitSwitch;
  
  public Backwards(DrivetrainSubsystem drive) {
    drivetrainSubsystem = drive;
    addRequirements(drivetrainSubsystem);
}
  
// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intializing......Running BACKWARDS.........");
    running = true;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Enter Execute BACKWARDS...............");    
      //running = true;
    long SinceHowLongRunning = System.currentTimeMillis() - startTime;

    if (!running || (DURRATION_IN_MILLISECONDS <= SinceHowLongRunning)){
      System.out.println("Shutting Down BACKWARDS................");
      shutDownInAuton(); 
      running = false;
      drivetrainSubsystem.drive(0,0);
    } else {
      System.out.println("Running BACKWARDS......................");
      drivetrainSubsystem.drive(ROTATION,SPEED);
      drivetrainSubsystem.drive(-.50,0);
      //Timer.stop(5);
      
    }
    //USE A BREAK; TO BREAK THE LOOP THE PROBLEM IS THAT ITS LOOPING AROUND AGAIN AND AGAIN
  }
  private void shutDownInAuton(){
    drivetrainSubsystem.drive(0,0);
    running = false;
    drivetrainSubsystem.m_drivetrain.setSafetyEnabled(false);
    System.out.println("ShutDownInAuton(BACKWARDS) EXIT..............");
  

    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End(BACKWARDS).................");
    running = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("IsFinished BACKWARDS................");
    return !running;
  }
}
