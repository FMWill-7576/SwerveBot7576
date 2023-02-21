// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase {
    Elevator s_elevator;
    private DoubleSupplier elevatorSup;
  /** Creates a new elevatorCommand. */
  public ElevatorCommand(
        Elevator s_elevator,
    DoubleSupplier elevatorSup) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_elevator = s_elevator;
    this.elevatorSup = elevatorSup;
    addRequirements(s_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 s_elevator.elevatorDrive(elevatorSup.getAsDouble());
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
