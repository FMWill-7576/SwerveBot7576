// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slider;

public class SliderCommand extends CommandBase {
    Slider s_Slider;
    private DoubleSupplier slideSup;
  /** Creates a new SliderCommand. */
  public SliderCommand(
        Slider s_Slider,
    DoubleSupplier slideSup) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Slider = s_Slider;
    addRequirements(s_Slider);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Slider.slideMotorConfig();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 s_Slider.slide(slideSup.getAsDouble());
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
