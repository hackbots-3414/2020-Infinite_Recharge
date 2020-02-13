package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    WPI_TalonFX leftMotor = new WPI_TalonFX(12);
    WPI_TalonFX rightMotor = new WPI_TalonFX(11);

    public Shooter(){
        super();
        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor);
        init();
    }

    public double getLeftShooterVelocity() {
        return leftMotor.getSelectedSensorVelocity();
    }

    public double getRightShooterVelocity() {
        return rightMotor.getSelectedSensorVelocity();
    }

    public double getAverageShooterVelocity() {
        return (getRightShooterVelocity() + getLeftShooterVelocity()) / 2;
    }

    public double getMeasurement() {
        return getAverageShooterVelocity();
    }

    public void init() {
        leftMotor.config_kP(0, 0.5);
        leftMotor.config_kI(0, 0);
        leftMotor.config_kD(0, 25);
        leftMotor.config_kF(0, 0.04);
    }

    public void setShooterVelocity(double setpoint) {
        setSetpoint(setpoint);
    }

    private void setSetpoint(double setpoint) {
        return;
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

}