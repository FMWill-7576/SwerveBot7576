package frc.robot.autos;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

public class OnlyGrip extends SequentialCommandGroup {
  public OnlyGrip(Swerve s_Swerve, Gripper s_Gripper ) {

    // An example trajectory to follow.  All units in meters.

    addCommands(
      new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      s_Gripper.run(() -> s_Gripper.auto()).withTimeout(1.1),
      new InstantCommand(() -> s_Gripper.stop()),
     new InstantCommand(() -> s_Swerve.drive(
            new Translation2d(0, 0),
            0,
            false,
            true))     
     );     
  }
}