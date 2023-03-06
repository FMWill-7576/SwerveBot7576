// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CimTest;

public class CimCommand extends CommandBase {
  CimTest s_CimTest;
  private DoubleSupplier cimSup;
  
  /** Creates a new CimCommand. */
  public CimCommand(
     CimTest s_CimTest,
  DoubleSupplier cimSup
    ) {
      this.s_CimTest = s_CimTest;
      addRequirements(s_CimTest);
      this.cimSup = cimSup;
      
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cimVal =
     MathUtil.applyDeadband(cimSup.getAsDouble(),0.05);
    s_CimTest.cimDrive(cimVal);
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
