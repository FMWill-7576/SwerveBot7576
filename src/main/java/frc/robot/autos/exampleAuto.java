package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class exampleAuto extends SequentialCommandGroup {
  public exampleAuto(Swerve s_Swerve) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.

    /* Trajectory testTrajectory =
         TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))),
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))),
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))),
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0)))
         ), config);    */

 

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))),
            // Pass through the  waypoints
            List.of(    
                new Translation2d(1.0, 0.01), 
                new Translation2d(2.5, 0.0)
                ),
            // end
            new Pose2d(4.0, 0.0, new Rotation2d(Math.toRadians(180))),
            config);
     
    Trajectory exampleTrajectory2 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction++
                new Pose2d(4.0, 0.0, new Rotation2d(Math.toRadians(180))),
                // Pass through these two  waypoints
                List.of(
                    new Translation2d(2.5, 0), 
                    new Translation2d(1.0, 0.01)
                    ),
                // End  where we started
                new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))),
                config);
                
    // Trajectory combinedTrajectory = exampleTrajectory.concatenate(exampleTrajectory2);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0.0,
            0.1,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.15),
            new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.15),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand2 =
    new SwerveControllerCommand(
        exampleTrajectory2,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.15),
        new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.15),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);

    addCommands(

     new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),

     swerveControllerCommand,

     new InstantCommand(() -> s_Swerve.drive(
            new Translation2d(0, 0),
            0,
            false,
            true)),

     new WaitCommand(2.5),

     new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory2.getInitialPose())),

     swerveControllerCommand2);
      
  }
}
