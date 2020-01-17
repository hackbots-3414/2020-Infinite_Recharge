/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.print.attribute.standard.JobPriority;
import javax.swing.text.Style;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveCommand extends CommandBase {
  public static final String RobotMap = null;
  /**
   * Creates a new DriveCommand.
   */
  private final DrivetrainSubsystem drivetrainSubsystem;
  private long startTime = 0;
  private boolean running= false;

  Joystick leftJoy  = new Joystick(0);
  JoystickButton toggleButton = new JoystickButton(leftJoy, 2);



  public DriveCommand(DrivetrainSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = subsystem;
    addRequirements(drivetrainSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    if(leftJoy.getRawButtonPressed(2)){
      startTime = System.currentTimeMillis();
      running = true;
    }

    long SinceHowLongRunning = System.currentTimeMillis() - startTime;

    if (!running || (3000 <= SinceHowLongRunning)){
  
     
      drivetrainSubsystem.drive(0,0);
      running = false;
    }
    else{
      drivetrainSubsystem.drive(1,0);
    }
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
