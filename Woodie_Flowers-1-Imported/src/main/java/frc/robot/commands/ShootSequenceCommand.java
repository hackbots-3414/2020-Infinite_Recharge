/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootSequenceCommand extends SequentialCommandGroup {
  /**
   * Creates a new SequenceCommand.
   */

  Shooter shooter;
  DrivetrainSubsystem drivetrain;

  public ShootSequenceCommand(BeltSubsyteem belt, DrivetrainSubsystem drivetrain, Shooter shooter, LEDSubsystem led,
     IntakeSubsystem intake, LimelightSubsystem limelight) {
    super();
    //addCommands(new AlignAndShootCommand(limelight, drivetrain, shooter), new
    //BeltShootCommand(belt));
    addCommands(new SpinUpCommand(shooter, limelight), new BeltShootCommand(belt,0.5));
    //addCommands(new TransportFullLEDParallelCommand(intake, belt, led), new ShooterLEDParallelCommand(shooter, led));
    this.shooter = shooter;
    this.drivetrain = drivetrain;
  }
 @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    SmartDashboard.putNumber("shooterVelocityRead", shooter.getAverageShooterVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    drivetrain.stop();
    super.end(interrupted);
    // new StopCommand(shooter, drivetrain);
  }
}
