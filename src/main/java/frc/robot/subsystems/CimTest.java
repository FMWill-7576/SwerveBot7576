// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CimTest extends SubsystemBase {
  public static double speedRate = 1.0;
  private VictorSPX cimMotor;
  private VictorSPX cimMotor2;
  /** Creates a new CimTest. */
  public CimTest() {
    //cimMotor = new VictorSPX(27);
   // cimMotor2 = new VictorSPX(30);
   // cimMotorConfig(); 
  
  }
  public void cimDrive(double value){
    //cimMotor.set(VictorSPXControlMode.PercentOutput, value);
    cimMotor2.set(VictorSPXControlMode.PercentOutput, value);
  }
  public void cimMotorConfig(){
    // cimMotor.configFactoryDefault();
    cimMotor2.configFactoryDefault();
    // cimMotor.enableVoltageCompensation(true);
    cimMotor2.enableVoltageCompensation(true);
   //  cimMotor.configVoltageCompSaturation(12.0);
    cimMotor2.configVoltageCompSaturation(12.0);
    // cimMotor.set(VictorSPXControlMode.PercentOutput,0.0 );
    cimMotor2.set(VictorSPXControlMode.PercentOutput,0.0 );
  }


  
  @Override
  public void periodic() {
   // SmartDashboard.putNumber("motor2",cimMotor2.getMotorOutputVoltage());
    //SmartDashboard.putNumber("motor1",cimMotor.getMotorOutputVoltage());
    // This method will be called once per scheduler run
  }
}
