// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
    public static double speedRate = 1.0;
    private VictorSPX gripperMotor1;
    private VictorSPX gripperMotor2;
    private PIDController gripperController1;
    private PIDController gripperController2;
  /** Creates a new Gripper. */
  public Gripper() { 
        gripperMotor1 = new VictorSPX(Constants.Gripper.gripperMotorID);
        gripperMotor2 = new VictorSPX(Constants.Gripper.gripperMotorID);
        gripperMotorConfig();
       }


  public void grip(VictorSPXControlMode controlMode, double gripperPercentage){
    gripperMotor1.set(controlMode, gripperPercentage);
    gripperMotor2.set(controlMode, gripperPercentage);
        } 

  public void gripperMotorConfig() { 
    gripperMotor1.configFactoryDefault();
   // CANSparkMaxUtil.setCANSparkMaxBusUsage(gripperMotor, Usage.kPositionOnly);
    //gripperMotor.setSmartCurrentLimit(Constants.gripper.gripperContinuousCurrentLimit);
    gripperMotor1.setInverted(Constants.Gripper.gripperInvert);
    gripperMotor1.setNeutralMode(Constants.Gripper.gripperNeutralMode);
    //integratedgripperEncoder.setVelocityConversionFactor(Constants.gripper.gripperConversionVelocityFactor);
    //integratedgripperEncoder.setPositionConversionFactor(Constants.gripper.gripperConversionPositionFactor);
    gripperController1.setP(Constants.Gripper.gripperKP);
    gripperController1.setI(Constants.Gripper.gripperKI);
    gripperController1.setD(Constants.Gripper.gripperKD);
    //gripperController.setFF(Constants.gripper.gripperKFF);
    gripperMotor1.enableVoltageCompensation(Constants.Gripper.voltageCompBoolean);
    gripperMotor1.configVoltageCompSaturation(Constants.Gripper.voltageComp);
    //gripperMotor.burnFlash();
    //integratedgripperEncoder.setPosition(0.0);

    gripperMotor2.configFactoryDefault();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(gripperMotor, Usage.kPositionOnly);
     //gripperMotor.setSmartCurrentLimit(Constants.gripper.gripperContinuousCurrentLimit);
     gripperMotor2.setInverted(Constants.Gripper.gripperInvert);
     gripperMotor2.setNeutralMode(Constants.Gripper.gripperNeutralMode);
     //integratedgripperEncoder.setVelocityConversionFactor(Constants.gripper.gripperConversionVelocityFactor);
     //integratedgripperEncoder.setPositionConversionFactor(Constants.gripper.gripperConversionPositionFactor);
     gripperController2.setP(Constants.Gripper.gripperKP);
     gripperController2.setI(Constants.Gripper.gripperKI);
     gripperController2.setD(Constants.Gripper.gripperKD);
     //gripperController.setFF(Constants.gripper.gripperKFF);
     gripperMotor2.enableVoltageCompensation(Constants.Gripper.voltageCompBoolean);
     gripperMotor2.configVoltageCompSaturation(Constants.Gripper.voltageComp);
     //gripperMotor.burnFlash();
     //integratedgripperEncoder.setPosition(0.0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
