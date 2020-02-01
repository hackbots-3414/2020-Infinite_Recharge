package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ShooterSubsystem extends PIDSubsystem {

    WPI_TalonFX motor1 = new WPI_TalonFX(10);

    public ShooterSubsystem(PIDController controller) {
        super(controller);
        // TODO Auto-generated constructor stub
    }

    @Override
    protected void useOutput(final double output, final double setpoint) {
        // TODO Auto-generated method stub

    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return getShooterVelocity();
    }

    public void setShooterVelocity(double setpoint) {
        setSetpoint(setpoint);
    }

    public double getShooterVelocity() {
        return motor1.getSelectedSensorVelocity();
    }

}