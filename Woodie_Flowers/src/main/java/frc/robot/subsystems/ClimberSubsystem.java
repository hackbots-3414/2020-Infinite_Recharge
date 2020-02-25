package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    TalonSRX elevator = new TalonSRX(0);
    TalonSRX winch = new TalonSRX(0);
}