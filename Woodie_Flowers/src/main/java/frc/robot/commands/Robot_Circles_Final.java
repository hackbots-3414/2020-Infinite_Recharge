/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Robot_Circles_Final extends SequentialCommandGroup {

  public Robot_Circles_Final(DrivetrainSubsystem drive) {
    addCommands(
    new Forward(drive),
    new Eight_LeftTurn(drive),
    new Eight_Forward(drive),
    new Eight_RightTurn(drive),
    new Forward(drive),
    new Eight_RightTurn(drive),
    new Eight_LeftTurn(drive)





    );
    
  }
}
