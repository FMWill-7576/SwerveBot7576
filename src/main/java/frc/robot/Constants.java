package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;



public final class Constants {
  
  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    //public static final int pigeonID = 6;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.585;
    // Distance between right and left wheels
    public static final double wheelBase = 0.585;
    // Distance between front and back wheels
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.03;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.025; 
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 1.91;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.82; // meters per second
    // public static final double maxSpeed = 5880.0 / 60.0 * 0.1633 * 0.1016 *Math.PI; // 5.1 meters per second
    //public static final double maxAngularVelocity = 11.5; 
    public static final double maxAngularVelocity = maxSpeed/Math.hypot(trackWidth/2,wheelBase/2); //11.96
    

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;



    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(82.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(18.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(317.9);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(245.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 1.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

       /*============================
                Vision
    ==============================*/

    /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     */
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.33, 0.0, 0.0), new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

    public static final double CAMERA_HEIGHT = 0.22;
    public static final double TAG_HEIGHT = 0.5; //TODO: temporary value

}
