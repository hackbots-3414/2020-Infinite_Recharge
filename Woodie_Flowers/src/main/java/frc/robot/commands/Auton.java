package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Auton extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    public Auton(DrivetrainSubsystem subsystem) {
      
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
        drivetrainSubsystem.drive();
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
    
      }
    
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return true;
      }
    }
    
}
