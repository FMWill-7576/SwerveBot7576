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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveBack;
import frc.robot.commands.DriveForward;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VictorArm;
import frc.robot.subsystems.Slider;


public class TaxiAndBalance extends SequentialCommandGroup {
  
  public TaxiAndBalance(Swerve s_Swerve, Gripper s_Gripper, VictorArm s_VictorArm, Slider s_Slider) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);




Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180.0))),
            // Pass through the  waypoints
            List.of(    
                new Translation2d(0.0, 0.01), 
                new Translation2d(1.5, 0.0)
                ),
            // end
            new Pose2d(2.0, 0.0, new Rotation2d(Math.toRadians(180.0))),
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
      /*   Commands.run(() -> s_VictorArm.armUp()).withTimeout(0.6).asProxy(),
       
        new InstantCommand(() -> s_VictorArm.victorDrive(0.0), s_VictorArm),
        s_Slider.run(() -> s_Slider.slideTesting2()).withTimeout(0.25),
        new WaitCommand(1.0),
        s_Slider.run(() -> s_Slider.slideTesting()).withTimeout(0.4) */

         s_Gripper.run(() -> s_Gripper.outake()),
        new WaitCommand(0.5),
        new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
        
        // new InstantCommand(() -> s_Swerve.invertGyro()),
       new InstantCommand(() -> s_Swerve.zeroGyro()),
       new DriveForward(s_Swerve),
       new WaitCommand(1.7),
      //new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
  
       //swerveControllerCommand
       new DriveBack(s_Swerve),
       new BalanceCommand(s_Swerve) 
      
   
     );
  
  }
}
