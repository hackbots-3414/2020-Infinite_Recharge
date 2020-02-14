package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    WPI_TalonFX leftMotor = new WPI_TalonFX(12);
    WPI_TalonFX rightMotor = new WPI_TalonFX(11);
    public static final double SHOOTER_VELOCITY = 16700;
    public static final double VELOCITY_OFFSET = 100;

    public Shooter() {
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

    public void init() {
        leftMotor.config_kP(0, 0.5);
        leftMotor.config_kI(0, 0);
        leftMotor.config_kD(0, 25);
        leftMotor.config_kF(0, 0.04);
    }

    public void shoot() {
        // TODO give this a good name
        System.out.println("inside shoot()------------------------");
        leftMotor.set(ControlMode.Velocity, SHOOTER_VELOCITY);
    }

    public boolean isReadyToShoot() {
        double currentShooterVelocity = leftMotor.getSelectedSensorVelocity();
        if (currentShooterVelocity >= SHOOTER_VELOCITY - VELOCITY_OFFSET) {
            shoot();
            return true;
            // TODO return true when shooter velocity is within
            // tolerance of desired velocity
            // command.isFinished() will call this
        }
        return false;
    }

    public void stop() {
        leftMotor.set(0.0);
    }

}