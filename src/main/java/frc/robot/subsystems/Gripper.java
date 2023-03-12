// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
    private Compressor pcmCompressor;
    Solenoid exampleSolenoidPCM;
  /** Creates a new Gripper. */
  public Gripper() {

    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    //pcmCompressor.enableDigital();
    //pcmCompressor.disable();

    exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    boolean enabled = pcmCompressor.isEnabled();
    boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    double current = pcmCompressor.getCurrent();
  }


  public void pistonTest(){
    exampleSolenoidPCM.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
