// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

public class ArmCommand extends CommandBase {
  Arm s_Arm;
  private DoubleSupplier armSup;
  VictorSPXControlMode controlMode;
  
  /** Creates a new ArmCommand. */
  public ArmCommand(
      Arm s_Sarm,
      DoubleSupplier armSup,
      VictorSPXControlMode controlMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Arm = s_Arm;
    addRequirements(s_Arm);
    this.armSup = armSup;
    this.controlMode = controlMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Arm.armDrive(controlMode, armSup.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
