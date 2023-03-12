// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Uses a PID and the gyroscope to balance the robot on the charger. */
public class BalanceCommandWithPid extends CommandBase {
  private final Swerve s_Swerve;
  private final PIDController balanceController = new PIDController(Constants.AutoConstants.Balancekp,0,Constants.AutoConstants.Balancekd);

  /**
   * Creates a new {@link BalanceCommand}.
   * 
   * @param subsystem The required subsystem.
   */
  public BalanceCommandWithPid(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    
    s_Swerve.drive(
      
      new Translation2d(balanceController.calculate(s_Swerve.getPitch(), 0), 
      0),
        0,
        true,
        true);
  } 
  
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(0,0), 0.0, true, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}