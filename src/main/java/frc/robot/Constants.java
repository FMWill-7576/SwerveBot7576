package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  public static final class Gripper {
    public static final int gripperMotorID = 99 ;
    public static final int pistonID = 2;
    public static final double gripperKP = 0.03 ;
    public static final double gripperKI = 0.0 ;
    public static final double gripperKD = 0.0 ;
    public static final double gripperKFF = 0.0 ;
    public static final  NeutralMode  gripperNeutralMode = NeutralMode.Brake;
    public static final boolean gripperInvert = false ;
    public static final double voltageComp = 12.0 ;
    public static final Boolean voltageCompBoolean = true ;
    public static final int gripperContinuousCurrentLimit = 40;

  }


  public static final class Arm {
    public static final int armMotorID = 99 ;
    public static final double armKP = 0.03 ;
    public static final double armKI = 0.0 ;
    public static final double armKD = 0.0 ;
    public static final double armKFF = 0.0 ;
    public static final  NeutralMode  armNeutralMode = NeutralMode.Brake;
    public static final boolean armInvert = false ;
    public static final double voltageComp = 12.0 ;
    public static final Boolean voltageCompBoolean = true ;
    public static final int armContinuousCurrentLimit = 40;

  }


  public static final class Slider {
  public static final int slideMotorID = 999;
  public static final double slideKP = 0.03;
  public static final double slideKI = 0.0;
  public static final double slideKD = 0.0;
  public static final double slideKFF = 0.0;
  public static final IdleMode slideNeutralMode = IdleMode.kBrake;
  public static final boolean slideInvert = false;
  public static final double voltageComp = 12.0;
  public static final int slideContinuousCurrentLimit = 40;
  
  }
  public static final class Swerve {
    public static final double stickDeadband = 0.07;

   
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
    public static final int driveContinuousCurrentLimit = 60;

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
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 2.48;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}
