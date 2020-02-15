package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;


public class All_In_One extends SequentialCommandGroup {
    public All_In_One(DrivetrainSubsystem drive){
     addCommands(
        // Drive forward the specified distance
        //new Backwa
        new Forward(drive),
        new Forward(drive),
        new Forward(drive),
        new Forward(drive),
        new Right_Turn(drive),
        new Right_Turn2(drive),
        new Forward(drive),
        new Forward(drive),
        new Forward(drive)
        
        
        
       

        //new Right_Turn(drive),
        //new Left_Turn(drive)
        
    
        
        

        
        

     );  
        // Drive backward the specified distance
    }




    
}