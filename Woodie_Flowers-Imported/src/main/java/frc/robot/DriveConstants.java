/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * Add your docs here.
 */
public class DriveConstants {
    public static final double ksVolts = 6.41;
    public static final double kvVoltSecondsPerMeter = 0.000988;
    public static final double kaVoltSecondsSquaredPerMeter = .0000487;
    public static final double kPDriveVel = 0.00197;
    public static final double kTrackwidthMeters = 3.3045780099373028;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);;
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kGyroReverse = 0;
	public static boolean kGyroReversed;

}
