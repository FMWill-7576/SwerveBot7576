// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;


public class Slider extends SubsystemBase {
  public static double speedRate = 1.0;
  private CANSparkMax slideMotor;
  private RelativeEncoder integratedSlideEncoder ;
  private SparkMaxPIDController slideController;
  public static double slidePosition;
  
  public Slider() {
 
   slideMotor = new CANSparkMax(54, MotorType.kBrushless);
   integratedSlideEncoder = slideMotor.getEncoder();
   slideController = slideMotor.getPIDController();
   slideMotorConfig();
  
  }
  public void slideDrive(double value){
    slideMotor.set(value);

  }
    public void armUp(){
      slideMotor.set(1.0);
    }

    public void armDown(){
     slideMotor.set(-1.0);
    }

    public void resetSlider(){
      integratedSlideEncoder.setPosition(0.0);
    }
  
  public void slideMotorConfig(){
    slideMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(slideMotor, Usage.kPositionOnly);
    slideMotor.setSmartCurrentLimit(50);
    slideMotor.setInverted(false);
    slideMotor.setIdleMode(IdleMode.kBrake);
    integratedSlideEncoder.setPositionConversionFactor(1/3);
    integratedSlideEncoder.setPosition(0);
    slideMotor.setSoftLimit(SoftLimitDirection.kReverse, -100);
    slideController.setP(0);
    slideController.setI(0);
    slideController.setD(0);
    slideController.setFF(0);
    slideMotor.enableVoltageCompensation(12.0);
    slideMotor.burnFlash();

  }

 
  @Override
  public void periodic() {
   slidePosition = integratedSlideEncoder.getPosition();
   SmartDashboard.putNumber("slidePosition",slidePosition);

    // This method will be called once per scheduler run
  }
}
