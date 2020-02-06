
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;


public class Right_Turn extends CommandBase {
  public static final String RobotMap = null;
  public static final long DURRATION_IN_MILLISECONDS = 1;
  public static final double SPEED = .15;
  public static final double ROTATION = 0;
  public static final double left = 0.0;
  public static final double right = -.15;
  //public static final long Angle_Remaining = 90;


  private final DrivetrainSubsystem drivetrainSubsystem;
  private long startTime = 0;
  private boolean running = false;

  public Right_Turn(DrivetrainSubsystem drive) {
    drivetrainSubsystem = drive;
    addRequirements(drivetrainSubsystem);
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Turning right ................");
    
    startTime = System.currentTimeMillis();
    running = true;
    long SinceHowLongRunning = System.currentTimeMillis() - startTime;

    if (!running ||(DURRATION_IN_MILLISECONDS <= SinceHowLongRunning)){
      System.out.println("Shutting Down right_turn...................");
      shutDownInAuton();

    }
    else{
      System.out.println("Running Right_turn............");
      drivetrainSubsystem.drive(SPEED, ROTATION);
      drivetrainSubsystem.drive(.5,.50);
      Timer.delay(5);
    }
    //if else{

   // }
  }
  private void shutDownInAuton(){
    drivetrainSubsystem.drive(0,0);
    running = false;
    drivetrainSubsystem.m_drivetrain.setSafetyEnabled(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    running = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !running;
  }
}
