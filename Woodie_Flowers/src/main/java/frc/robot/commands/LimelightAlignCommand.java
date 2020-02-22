package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightAlignCommand extends CommandBase {

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
        double angle_tolerance = 0.5;
        // double ta = limelight.getTargetArea();

        if (tx > -1 * angle_tolerance && tx < angle_tolerance) {

            drivetrain.tankDrive(0, 0);
            return true;
        } else {

            // double kp = -0.025f;
            double heading_error = tx;
            double base = 0.25;
            double throttleFloor = 0.125;
            double throttlePercent = 0.0;
            double angleBias = 1.5;
            // Equation to perform an inverse expontation decay of the throttle response
            double magnitude = base - 1 / Math.exp(Math.abs(heading_error + angleBias));

            throttlePercent = Math.max(Math.abs(magnitude), throttleFloor);
            throttlePercent = Math.copySign(throttlePercent, heading_error) * -1;

            /*
             * double steering_adjust = kp * heading_error;
             * System.out.println("this is steering_adjust " + steering_adjust);
             * steering_adjust =
             * Math.copySign(Math.max(steering_adjust,.20),steering_adjust);
             * System.out.println("this is steering_adjust " +steering_adjust);
             */

            double left = 0.0;
            left += throttlePercent;// steering_adjust; originally subtract
            double right = 0.0;
            right -= throttlePercent;// steering_adjust;

            // //New alignmnet code
            // if (tx > 0) {
            // //Pivot Left
            // left = 0.2;
            // right = -0.2;
            // } else {
            // //Pivot right
            // left = -0.2;
            // right = 0.2;
            // }
            drivetrain.tankDrive(left, right);
            return false;
        }

    }

}