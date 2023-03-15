// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class driveBack extends CommandBase {
  static double initialRoll;
  private final Swerve s_Swerve;
  /** Creates a new driveBack. */
  public driveBack(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   initialRoll = s_Swerve.getRoll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.drive(
      
      new Translation2d(-0.65, 
      0),
        0,
        true,
        true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(s_Swerve.getRoll()) > initialRoll + 1.5 );
  }
}
