// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
   public static double speedRate = 1.0;
private VictorSPX armMotor;
   private PIDController armController;
  /** Creates a new Arm. */
  public Arm() {
    armMotor = new VictorSPX(Constants.Arm.armMotorID);
    armMotorConfig();
  }

  public void armDrive(VictorSPXControlMode controlMode, double armPercentage){
    armMotor.set(controlMode, armPercentage);
        } 

  public void armMotorConfig() { 
    armMotor.configFactoryDefault();
   // CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly);
    //armMotor.setSmartCurrentLimit(Constants.Arm.armContinuousCurrentLimit);
    armMotor.setInverted(Constants.Arm.armInvert);
    armMotor.setNeutralMode(Constants.Arm.armNeutralMode);
    //integratedarmEncoder.setVelocityConversionFactor(Constants.arm.armConversionVelocityFactor);
    //integratedarmEncoder.setPositionConversionFactor(Constants.arm.armConversionPositionFactor);
    armController.setP(Constants.Arm.armKP);
    armController.setI(Constants.Arm.armKI);
    armController.setD(Constants.Arm.armKD);
    //armController.setFF(Constants.Arm.armKFF);
    armMotor.enableVoltageCompensation(Constants.Arm.voltageCompBoolean);
    armMotor.configVoltageCompSaturation(Constants.Arm.voltageComp);
    //armMotor.burnFlash();
    //integratedarmEncoder.setPosition(0.0);
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
