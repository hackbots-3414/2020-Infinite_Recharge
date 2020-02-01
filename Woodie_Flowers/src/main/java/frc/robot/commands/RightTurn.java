package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RightTurn extends SequentialCommandGroup {
    public RightTurn(DrivetrainSubsystem drive){
     addCommands(
        // Drive forward the specified distance
        new NotDriveTrain(drive),

        // Drive backward the specified distance
        new Right_Turn(drive));
        //new Right_Turn(drive,0.1,0.5,100));

    }




    
}