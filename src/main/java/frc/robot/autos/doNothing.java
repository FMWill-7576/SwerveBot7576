package frc.robot.autos;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class doNothing extends SequentialCommandGroup {
  public doNothing(Swerve s_Swerve) {

    // An example trajectory to follow.  All units in meters.
    
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0.0,
            0.1,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(
     new InstantCommand(() -> s_Swerve.drive(
            new Translation2d(0, 0),
            0,
            false,
            true))     
     );     
  }
}