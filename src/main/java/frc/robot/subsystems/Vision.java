package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
@SuppressWarnings("unused")

public class Vision extends SubsystemBase{

    public double tx;
    public double ty;
    public double id;
    public double tv;
    public double ta;
    public double[] botpose;
    LimelightHelpers.LimelightResults llresults;

    public Vision() { 
      //  LimelightHelpers.setCameraMode_Driver("limelight");
       // LimelightHelpers.setPipelineIndex("limelight", 1);
        
        
    }

    public double getYDistance(double y) {

        return((Constants.TAG_HEIGHT -Constants.CAMERA_HEIGHT)/(Math.tan((y) * Math.PI/180)));

    }

    public double getXDistance(double x, double y) {

        return(getYDistance(y) / Math.tan(x * Math.PI / 180));

    }

    public double getTheta() {

        return tx;

    }

    @Override
    public void periodic() {

     /* double tx = LimelightHelpers.getTX("");
     double ty = LimelightHelpers.getTY("");
     boolean hasTarget = LimelightHelpers.getTV("");
     double id = LimelightHelpers.getFiducialID("");
     double[] emptyArray = new double[0];
     botpose = LimelightHelpers.getBotPose("");
     llresults = LimelightHelpers.getLatestResults(""); */

        /* SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putBoolean("has target", hasTarget);
        SmartDashboard.putNumber("tag id", id);

        SmartDashboard.putNumber("LL x-dist", getXDistance(tx, ty));
        SmartDashboard.putNumber("LL y-dist", getYDistance(ty));
        SmartDashboard.putNumber("LL theta", getTheta()); */

    }
    
}