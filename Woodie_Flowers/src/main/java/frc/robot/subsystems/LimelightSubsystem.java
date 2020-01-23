package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {
        super();
    }

    public double getVerticalOffset() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").setNumber(0);
        return ty;
    }

    public double getHorizontalOffset() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").setNumber(0);
        return tx;
    }

    public double getTargetArea() {
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").setNumber(0);
        return ta;
    }

    public double getSkew(){
        double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").setNumber(0);
        return ts;
    }

    public double getHorizonalSideLength() {
        double thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").setNumber(0);
        return thor;
    }

    public double getVerticalSideLength() {
        double tver = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tver").setNumber(0);
        return tver;
    }

    public double getLongestSide() {
        double tlong = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").setNumber(0);
    }

    public double getShortestSide() {
        double tshort = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").setNumber(0);
    }


}
