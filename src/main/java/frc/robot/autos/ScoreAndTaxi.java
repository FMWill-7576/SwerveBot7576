// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;




import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveForward;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VictorArm;
import frc.robot.subsystems.Slider;


public class ScoreAndTaxi extends SequentialCommandGroup {
  
  public ScoreAndTaxi(Swerve s_Swerve, Gripper s_Gripper, VictorArm s_VictorArm, Slider s_Slider) {

            

    addCommands(
        // Commands.run(() -> s_VictorArm.armUp()).withTimeout(0.6),
       
      //  new InstantCommand(() -> s_VictorArm.victorDrive(0.0), s_VictorArm),
        //s_Slider.run(() -> s_Slider.slideTesting2()).withTimeout(0.25),
       // new WaitCommand(1.0),
       // s_Slider.run(() -> s_Slider.slideTesting()).withTimeout(0.4) ,

        s_Gripper.run(() -> s_Gripper.outake()).withTimeout(0.9),
       // s_Gripper.run(() -> s_Gripper.stop()),
        
         new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
        
        // new InstantCommand(() -> s_Swerve.invertGyro()),
       new InstantCommand(() -> s_Swerve.zeroGyro()),
       new DriveForward(s_Swerve).withTimeout(3),
      // new InstantCommand( () -> s_Swerve.drive
      // (new Translation2d(0.35,0).times(Constants.Swerve.maxSpeed),
       // 0, true, true)).withTimeout(2.0),
        new InstantCommand( () -> s_Swerve.drive(new Translation2d( 0,0), 0, true, true))
    
     );
  
  }
}
