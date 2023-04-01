package frc.robot.subsystems;
import frc.robot.NavX.*;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
  private final AHRS gyro =  new AHRS(SPI.Port.kMXP, (byte)50);

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
    try {
      Thread.sleep(1000); // waiting for 1 second for the navx to complete the calibration before resetting the yaw
      gyro.reset();
      invertGyro();
  } catch (InterruptedException ex) {
    Thread.currentThread().interrupt();
  } 

  
 // Timer.delay(1.0);
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
 // Timer.delay(1.0);

   swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
  
    
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getYaw()
                    )
                : new ChassisSpeeds(
                  translation.getX(), 
                  translation.getY(), 
                  rotation)
                  );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }
  

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }
 
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

 public void xLock(){ // put the wheels in an "x" formation to prevent being pushed
   SwerveModuleState[] swerveModuleStates2 = {
  new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
  new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)),
  new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)),
  new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
};
 for (SwerveModule mod : mSwerveMods) {
   mod.setDesiredStateForXlock(swerveModuleStates2[mod.moduleNumber], true);
} 
 }

 public void invertGyro(){
  gyro.setAngleAdjustment(0.0);
 }

  public void zeroGyro() {
    gyro.reset();
  }

  public double getTilt(){
    double pitch = gyro.getPitch();
    double roll = gyro.getRoll();
    if ((pitch+roll) >=0) {
      return Math.sqrt(pitch*pitch + roll*roll);
    } else {

    
    return  -Math.sqrt(pitch*pitch + roll*roll);
    }
  }

 
public static double speedRateSwerve = 1.0; 


 public void incSpeed() {
 /*  if(speedRateSwerve < 0.95 ) { 
  speedRateSwerve = speedRateSwerve + 0.1;
} */
speedRateSwerve = 1.0;
}

public void decSpeed() {
 /* if(speedRateSwerve > 0.15 ) { 
  speedRateSwerve = speedRateSwerve - 0.1;
} */
speedRateSwerve = 0.2;
}

public double getPitch() {
 return gyro.getPitch();
 }

public double getRoll() {
  return gyro.getRoll();
}

public Rotation2d getYaw() {
  //return gyro.getRotation2d();
   return Rotation2d.fromDegrees(-gyro.getAngle());
}


public void resetModulesToAbsolute(){
   for(SwerveModule mod : mSwerveMods){
       mod.resetToAbsolute();
   } 
}
 

  @Override
  public void periodic() {
    for(SwerveModule mod : mSwerveMods){
      if  (Math.abs(mod.integratedAngleEncoder.getPosition() - mod.getCanCoder().getDegrees()) > 5.0)
      mod.resetToAbsolute();
  } 
   
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Robot Heading",(-gyro.getAngle()));
    //SmartDashboard.putString("Robot Location", getPose().getRotation().toString());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("Speed Rate", speedRateSwerve);
    SmartDashboard.putNumber("pitch", (gyro.getPitch()));
    SmartDashboard.putNumber("roll", gyro.getRoll());
    SmartDashboard.putNumber("adjustment", gyro.getAngleAdjustment());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
          //SmartDashboard.putString(
          //"Mod " + mod.moduleNumber + " Position", mod.getPosition().toString());
    }
  }

 
}
