// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BalanceCommand2;
import frc.robot.commands.DriveBack;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveForward2;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VictorArm;
import frc.robot.subsystems.Slider;


public class ScoreAndBalance extends SequentialCommandGroup {
  
  public ScoreAndBalance(Swerve s_Swerve, Gripper s_Gripper, VictorArm s_VictorArm, Slider s_Slider) {

          

    addCommands(
        /* Commands.run(() -> s_VictorArm.armUp()).asProxy().withTimeout(0.6),
       
        new InstantCommand(() -> s_VictorArm.victorDrive(0.0), s_VictorArm),*/
        //s_Slider.run(() -> s_Slider.slideTesting2()).withTimeout(0.25),
       // new WaitCommand(1.0),
       // s_Slider.run(() -> s_Slider.slideTesting()).withTimeout(0.4) ,

       s_Gripper.run(() -> s_Gripper.outake()).withTimeout(0.9),
       new InstantCommand(() -> s_Gripper.stop()),
        
       new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
       new InstantCommand(() -> s_Swerve.zeroGyro()),
       new DriveForward2(s_Swerve),
       new WaitCommand(1.0),
       new BalanceCommand2(s_Swerve)
       
   
     );
  
  }
}
