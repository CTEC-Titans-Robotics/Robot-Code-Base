package frc8768.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    NetworkTable handle;
    public double x;
    public double y;
    public double area;

    public void tick() {
        pullVars();
        getDistance();
    }

    private void pullVars() {
        x = handle.getEntry("tx").getDouble(0.0);
        y = handle.getEntry("ty").getDouble(0.0);
        area = handle.getEntry("ta").getDouble(0.0);
    }

    public double getDistance() {
        double offsetAngle = 11;
        double robotCameraHeight = 28.25;
        double goalHeight = 104;

        double angleToGoalRadians = (y + offsetAngle) * (Math.PI / 180.0);

        double distance;
        distance = (goalHeight - robotCameraHeight)/Math.tan(angleToGoalRadians);

        return distance;
    }

    public void init() {
        if(handle == null) {
            handle = NetworkTableInstance.getDefault().getTable("limelight");
        }

        x = handle.getEntry("tx").getDouble(0.0);
        y = handle.getEntry("ty").getDouble(0.0);
        area = handle.getEntry("ta").getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
}
