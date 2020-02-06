/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;



import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.PIDNavXDrive;
import frc.robot.subsystems.Utilities;
import frc.robot.commands.TurnDotEXE;;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveDotEXE extends PIDCommand {
  /**
   * Creates a new DriveDotEXE.
   */
  int distan;
  public double okBoomer;
  PIDNavXDrive drivetrain;
  Utilities values;
  double m_tolerance;
  public DriveDotEXE(int distance,double speed,double tolerance,PIDNavXDrive driver) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          

        });
        okBoomer = speed;
        drivetrain = driver;
        values = new Utilities();
        distan = distance;
        m_tolerance = tolerance;


        getController().setD(values.k_DDrive);
        getController().setI(values.k_IDrive);
        getController().setP(values.k_PDrive);

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public void initialize() {
    drivetrain.enable();
    super.initialize();
    values.abruptStop =false;
    //CommandScheduler.getInstance().schedule( new TurnDotEXE(drivetrain, 0, 0 ));
  }
  @Override
  public void execute() {

    drivetrain.robotDrive(okBoomer, 0);

    System.out.println("right encoder values : " + drivetrain.getEncoderRight());
    System.out.println("left encoder values : " + -drivetrain.getEncoderLeft());
    super.execute();
    //CommandScheduler.getInstance().schedule( new TurnDotEXE(drivetrain, 0, 0 ));
    //10,000 = 17 inches
    //20,000 = 16 inches
  }
  @Override
  public boolean isFinished() {
    if(drivetrain.getMeasurement()>m_tolerance){
      drivetrain.setInterupted(true);
      drivetrain.setPreviousDistance(drivetrain.getEncoderRight() + drivetrain.getPreviousDistance());
      drivetrain.setPreviousAngle(drivetrain.getMeasurement());
      drivetrain.setLevel(true);
      return true;
    }
    if(drivetrain.getMeasurement()<-m_tolerance){
      drivetrain.setInterupted(true);
      drivetrain.setPreviousDistance(drivetrain.getEncoderRight() + drivetrain.getPreviousDistance());
      drivetrain.setPreviousAngle(drivetrain.getMeasurement());
      drivetrain.setLevel(true);
      return true;
    }

    if(-drivetrain.getEncoderLeft()> distan && drivetrain.getEncoderRight() > distan && distan>0){
      drivetrain.disable();
      drivetrain.robotDrive(0.0, 0.0);
      System.out.println("isFinished = true");
      values.atSetPoint = true;
      return true;
    }
    if(-drivetrain.getEncoderLeft()< distan && drivetrain.getEncoderRight() < distan && distan<0){
      drivetrain.disable();
      drivetrain.robotDrive(0.0, 0.0);
      System.out.println("isFinished = true");
      values.atSetPoint = true;
      return true;
    }
    else{
      return false;
    }
  }
}
