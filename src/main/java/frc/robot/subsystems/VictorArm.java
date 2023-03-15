// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VictorArm extends SubsystemBase {
  
  private VictorSPX victor1;
  private VictorSPX victor2;
  private VictorSPX victor3;
  private VictorSPX victor4;
  private MotorControllerGroup victors;
  /** Creates a new VictorArm. */
  public VictorArm() {
     victor1  = new VictorSPX(16);
     victor2 = new VictorSPX(30);
    //victor3 = new WPI_VictorSPX(18);
    //victor4 = new WPI_VictorSPX(19);
    //victors = new MotorControllerGroup(victor1,victor2);
    victorConfig();    
    
  }
  public void victorConfig(){
victor1.configFactoryDefault();
victor1.enableVoltageCompensation(true);
victor1.configVoltageCompSaturation(12.0);
victor1.setInverted(false);
victor1.setNeutralMode(NeutralMode.Brake);
    
victor2.configFactoryDefault();
victor2.enableVoltageCompensation(true);
victor2.configVoltageCompSaturation(12.0);
victor2.setInverted(false);
victor2.setNeutralMode(NeutralMode.Brake);
    
/* victor3.configFactoryDefault();
victor3.enableVoltageCompensation(true);
victor3.configVoltageCompSaturation(12.0);
victor3.setInverted(true);
victor3.setNeutralMode(NeutralMode.Brake);

victor4.configFactoryDefault();
victor4.enableVoltageCompensation(true);
victor4.configVoltageCompSaturation(12.0);
victor4.setInverted(true);
victor4.setNeutralMode(NeutralMode.Brake); */
    }
public void victorDrive(double power){ 
victor1.set(VictorSPXControlMode.PercentOutput, power);
victor2.set(VictorSPXControlMode.PercentOutput, power);
}

public void victorTest(){
  victor1.set(VictorSPXControlMode.PercentOutput, 0.15);
  victor2.set(VictorSPXControlMode.PercentOutput, 0.15);

}
public void victorTest2(){
  victor1.set(VictorSPXControlMode.PercentOutput, -0.08);
  victor2.set(VictorSPXControlMode.PercentOutput, -0.08);
}
  

  @Override
  public void periodic() {



    // This method will be called once per scheduler run
  }
}
