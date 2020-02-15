package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightAlignCommand extends CommandBase {

    private final LimelightSubsystem limelight;
    private final DrivetrainSubsystem drivetrain;

    public LimelightAlignCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
        super();
        System.out.println("inside LimelightAlignCommand");
        addRequirements(limelight);
        addRequirements(drivetrain);
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    public void execute() {
        System.out.println("inside execute()");

    }

    public boolean isFinished() {
        double tx = limelight.getHorizontalOffset();
        double ta = limelight.getTargetArea();
        System.out.println("this is tx: " + tx);

        if (tx > -0.3 && tx < 0.3) {
            System.out.println("The bot is stopped!");
            drivetrain.tankDrive(0, 0);
            return true;
        } else {
            System.out.println("not aligned inside else block");
            // double kp = -0.025f;
            double heading_error = tx;
            double base = 0.25;
            double throttleFloor = 0.175;
            double throttlePercent = 0.0;
            double angleBias = 1.5;
            double magnitude = base - 1 / Math.exp(Math.abs(heading_error + angleBias));
            System.out.println("this is magnitude " + magnitude);
            throttlePercent = Math.max(magnitude, throttleFloor);
            throttlePercent = Math.copySign(throttlePercent, heading_error) * -1;

            System.out.println("this is throttlePercent " + throttlePercent);
            /*
             * double steering_adjust = kp * heading_error;
             * System.out.println("this is steering_adjust " + steering_adjust);
             * steering_adjust = Math.copySign(Math.max(steering_adjust,.20),
             * steering_adjust); System.out.println("this is steering_adjust " +
             * steering_adjust);
             */
            double left = 0.0;
            left -= throttlePercent;// steering_adjust; originally subtract
            double right = 0.0;
            right += throttlePercent;// steering_adjust;

            drivetrain.tankDrive(left, right);
            return false;
        }

    }
    
}