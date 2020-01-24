package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightAlignCommand extends CommandBase{

    private final LimelightSubsystem limelight;
    private final DrivetrainSubsystem drivetrain;

    public LimelightAlignCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
        super();
        addRequirements(limelight);
        addRequirements(drivetrain);
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    public void execute() {
       
        

    }

    public boolean isFinished() {
        double tx = limelight.getHorizontalOffset();
        double ta = limelight.getTargetArea();
        
        if(tx > -0.3 && tx < 0.3) {
            drivetrain.tankDrive(0, 0);
            return true;
        }else {
            float kp = -0.1f;
            float heading_error = tx;
            steering_adjust = kp * tx;
    
            left += steering_adjust;
            right -= steering_adjust;
    
            drivetrain.tankDrive(left, right);
            return false;
        }
        
    }
}