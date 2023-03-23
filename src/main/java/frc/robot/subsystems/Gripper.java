// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private VictorSPX gripMotor;
   // private Compressor pcmCompressor;
   // Solenoid exampleSolenoidPCM;
  /** Creates a new Gripper. */
  public Gripper() {
    gripMotor = new VictorSPX(20);
    //pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    //pcmCompressor.enableDigital();
    //pcmCompressor.disable();

   // exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    //boolean enabled = pcmCompressor.isEnabled();
    //boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    //double current = pcmCompressor.getCurrent();
  }

  public void stop(){
    gripMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }
  public void   intake(){
    gripMotor.set(VictorSPXControlMode.PercentOutput, 0.65);
  }

  public void   outake(){ 
    gripMotor.set(VictorSPXControlMode.PercentOutput, -0.65);
  }

  public void gripConfig(){
gripMotor.configFactoryDefault();
gripMotor.enableVoltageCompensation(true);
gripMotor.configVoltageCompSaturation(12.0);
gripMotor.setInverted(false);
gripMotor.setNeutralMode(NeutralMode.Brake);
}

  /* public void pistonTest(){
    exampleSolenoidPCM.toggle();
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
