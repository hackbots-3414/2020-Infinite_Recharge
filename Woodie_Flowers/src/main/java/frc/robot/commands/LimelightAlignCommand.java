package frc.robot.commands;

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
        double tx = limelight.getHorizontalOffset();
        double ta = limelight.getTargetArea();
        

    }

    public boolean isFinished() {

    }
}