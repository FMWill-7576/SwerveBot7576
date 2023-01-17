package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase{

    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry tid;
    private NetworkTableEntry tv;
    private NetworkTable table;
    private double x;
    private double y;
    private double id;
    private double v;
    private NetworkTableEntry tbotpose;
    private double[] botpose;

    public Vision() {

        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");
        tbotpose = table.getEntry("botpose");

    }

    public double getYDistance(double y) {

        return((Constants.TAG_HEIGHT -Constants.CAMERA_HEIGHT)/(Math.tan((y) * Math.PI/180)));

    }

    public double getXDistance(double x, double y) {

        return(getYDistance(y) / Math.tan(x * Math.PI / 180));

    }

    public double getTheta() {

        return x;

    }

    public boolean hasTarget() {

        if(v == 0) {
            return false;
        } else if(v == 1.0) {
            return true;
        } else {
            return false;
        }

    }

    public double getID() {

        return id;

    }

    public Pose3d getTagPose(double tagID) {

        return new Pose3d(1, 2, 0.5, new Rotation3d(0, 0, 0));

    }

    public double[] getBotPose() {

        return botpose;

    }

    @Override
    public void periodic() {

        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        v = tv.getDouble(0);
        id = tid.getDouble(100);
        double[] emptyArray = new double[0];
        botpose = tbotpose.getDoubleArray(emptyArray);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("has target", v);
        SmartDashboard.putNumber("tag id", id);

        SmartDashboard.putNumber("LL x-dist", getXDistance(x, y));
        SmartDashboard.putNumber("LL y-dist", getYDistance(y));
        SmartDashboard.putNumber("LL theta", getTheta());

    }
    
}