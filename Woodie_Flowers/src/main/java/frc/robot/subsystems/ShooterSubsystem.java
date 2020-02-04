package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ShooterSubsystem extends PIDSubsystem {

    WPI_TalonFX leftMotor = new WPI_TalonFX(12);
    WPI_TalonFX rightMotor = new WPI_TalonFX(11);

    public ShooterSubsystem(PIDController controller) {
        super(controller);
        controller.setTolerance(Double.POSITIVE_INFINITY, 180);
        controller.enableContinuousInput(-11000, 11000);
        // TODO Auto-generated constructor stub
    }

    @Override
    protected void useOutput(final double output, final double setpoint) {
        // TODO Auto-generated method stub
        leftMotor.set(output);
        rightMotor.set(-output);
        System.out.println("this is the output: " + output);
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return getAverageShooterVelocity();
    }

    public void setShooterVelocity(double setpoint) {
        setSetpoint(setpoint);
    }

    public double getLeftShooterVelocity() {
        return leftMotor.getSelectedSensorVelocity();
    }

    public double getRightShooterVelocity() {
        return -rightMotor.getSelectedSensorVelocity();
    }

    public double getAverageShooterVelocity(){
        double averageShooterVelocity = getLeftShooterVelocity() + getRightShooterVelocity() / 2;
        return averageShooterVelocity;
    }

    public boolean atSetpoint(){
       return getController().atSetpoint();
    }

    public double getSetpoint(){
        return getController().getSetpoint();
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

    public void runLeftMotor(){
        leftMotor.set(ControlMode.PercentOutput, 0.02);
    }

}