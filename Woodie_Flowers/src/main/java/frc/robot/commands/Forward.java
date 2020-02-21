/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;


public class Forward extends CommandBase {
  public static final String RobotMap = null;
  public static final long DURATION_IN_MILLISECONDS = 2800;
  public static final double SPEED = .5;
  public static final double ROTATION = 0;
  /**
   * Creates a new DriveCommand.
   */
  private final DrivetrainSubsystem drivetrainSubsystem;
  private long startTime = 0;
  private boolean running= false;

  Joystick rightJoy  = new Joystick(0);
  JoystickButton toggleButton = new JoystickButton(rightJoy, 3);


  public Forward(DrivetrainSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = subsystem;
    addRequirements(drivetrainSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Forward: Intializing......Running.........");
    
    running = true;
    startTime = System.currentTimeMillis();
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Forward: Enter Execute...............");
    //if(rightJoy.getRawButtonPressed(3)){}
    long SinceHowLongRunning = System.currentTimeMillis() - startTime;

    if (!running || (DURATION_IN_MILLISECONDS <= SinceHowLongRunning)){
      System.out.println("Forward: Shutting Down................");
      shutDownInAuton(); 
    } else {
      System.out.println("Forward: Running......................");
      drivetrainSubsystem.drive(SPEED,ROTATION);
    }
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