// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

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
import frc.robot.Constants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.driveBack;
import frc.robot.subsystems.Swerve;


public class AdvancedAuto extends SequentialCommandGroup {
  
  public AdvancedAuto(Swerve s_Swerve) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);




Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))),
            // Pass through the  waypoints
            List.of(    
                new Translation2d(1.0, 0.01), 
                new Translation2d(2.0, 0.0)
                ),
            // end
            new Pose2d(4.0, 0.0, new Rotation2d(Math.toRadians(0.0))),
            config);
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

            
    addCommands(
      
      new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
 
     // swerveControllerCommand,
      new driveBack(s_Swerve),
      new BalanceCommand(s_Swerve)
  
     );
  
  }
}
