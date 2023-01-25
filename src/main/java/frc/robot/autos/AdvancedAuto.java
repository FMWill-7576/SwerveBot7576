// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AdvancedAuto extends SequentialCommandGroup {
  /** This is an exaple of what an advanced autonomous routine could look like this season!
   * Score the preloaded cube, then pick up another cube and score it, then drive onto the charging station and self-balance
   */
  public AdvancedAuto() {
    addCommands(new exampleAuto(null));
  
  }
}
