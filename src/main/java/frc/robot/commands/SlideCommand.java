// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slider;

public class SlideCommand extends CommandBase {
  Slider s_CimTest;
  private DoubleSupplier slideSup;
  
  /** Creates a new CimCommand. */
  public SlideCommand(
     Slider s_Slider,
  DoubleSupplier slideSup
    ) {
      this.s_CimTest = s_Slider;
      addRequirements(s_Slider);
      this.slideSup = slideSup;
      
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double slideVal =
     MathUtil.applyDeadband(slideSup.getAsDouble(),0.05);
    s_CimTest.slideDrive(slideVal);
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
