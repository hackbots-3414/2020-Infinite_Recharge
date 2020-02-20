/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;


import java.time.Duration;

import javax.swing.text.Style;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;


public class Eight_2ndLeftTurn extends CommandBase {
  public static final String RobotMap = null;
  public static final long DURRATION_IN_MILLISECONDS = 500;
  public static final double SPEED = 0;
  public static final double ROTATION = .5;
  //public static final double seconds = 5;
  //public static final double left = 0.0;
 // public static final double right = -.15;
  //public static final long Angle_Remaining = 90;


  private final DrivetrainSubsystem drivetrainSubsystem;
  private long startTime = 0;
  private boolean running = false;

  

  public Eight_2ndLeftTurn(DrivetrainSubsystem drive) {
    drivetrainSubsystem = drive;
    addRequirements(drivetrainSubsystem);
}
  

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Eight_Left_Turn: Intializing......Running.........");
    running = true;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Eight_Left_Turn: Enter Execute...............");    
      //running = true;
    long SinceHowLongRunning = System.currentTimeMillis() - startTime;

    if (!running || (DURRATION_IN_MILLISECONDS <= SinceHowLongRunning)){
      System.out.println("Eight_Left_Turn: Shutting Down................");
      shutDownInAuton(); 
      running = false;
      drivetrainSubsystem.drive(0,0);
    } else {
      System.out.println("Eight_Left_Turn: Running......................");
      drivetrainSubsystem.drive(ROTATION,SPEED);
      drivetrainSubsystem.drive(.50,-.50);
      //Timer.stop(5);
      
    }
    //USE A BREAK; TO BREAK THE LOOP THE PROBLEM IS THAT ITS LOOPING AROUND AGAIN AND AGAIN
  }
  private void shutDownInAuton(){
    drivetrainSubsystem.drive(0,0);
    running = false;
    drivetrainSubsystem.m_drivetrain.setSafetyEnabled(false);
    System.out.println("Eight_Left_Turn: ShutDownInAuton() EXIT..............");
  

    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Eight_Left_Turn: End().................");
    running = false;
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Eight_Left_Turn: IsFinished................");
    return !running;
    
  }
  
}


