// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;




import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveForward;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;



public class ScoreAndTaxi extends SequentialCommandGroup {
  
  public ScoreAndTaxi(Swerve s_Swerve, Gripper s_Gripper, Arm s_Arm) {

            

    addCommands(
     
    s_Arm.run(() -> s_Arm.armCone()).withTimeout(1.5),
       s_Gripper.run(() -> s_Gripper.auto()).withTimeout(0.9),
       new InstantCommand(() -> s_Gripper.stop()),
    
     new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
    
       // s_Gripper.run(() -> s_Gripper.stop()),
        
         new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
        
        // new InstantCommand(() -> s_Swerve.invertGyro()),
       new InstantCommand(() -> s_Swerve.zeroGyro()),
       new DriveForward(s_Swerve).withTimeout(3.0),
      // new InstantCommand( () -> s_Swerve.drive
      // (new Translation2d(0.35,0).times(Constants.Swerve.maxSpeed),
       // 0, true, true)).withTimeout(2.0),
        new InstantCommand( () -> s_Swerve.drive(new Translation2d( 0,0), 0, true, true))
    
     );
  
  }
}
