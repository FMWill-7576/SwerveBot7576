// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.robot.Robot;

public class VictorArm extends SubsystemBase {
  
  private VictorSPX bottomLeft;
  private VictorSPX bottomRight;
  private VictorSPX topLeft;
  private VictorSPX topRight;
  private CANCoder  armCoder;
  public double kG;
  public double upValue;
  public double downValue;
  /** Creates a new VictorArm. */
  public VictorArm() {
     bottomLeft  = new VictorSPX(16);
     bottomRight = new VictorSPX(17);
    topLeft = new VictorSPX(18);
    topRight = new VictorSPX(19);
    //armCoder = new CANCoder(23);
    victorConfig();
    //configArmCoder();
    kG = 0.225;
    //kG = 0.19;
    //kG = 0.121;
   //kG = 0.15;
   // kG = 0.0;
    upValue = 0.38;
    //upValue = 0.25;
    downValue = 0.03;
    //downValue = -0.05;
  }
  public void victorConfig(){
bottomLeft.configFactoryDefault();
bottomLeft.enableVoltageCompensation(true);
bottomLeft.configVoltageCompSaturation(12.0);
bottomLeft.setInverted(false);
bottomLeft.setNeutralMode(NeutralMode.Brake);
bottomLeft.set(VictorSPXControlMode.PercentOutput, 0);
    
bottomRight.configFactoryDefault();
bottomRight.enableVoltageCompensation(true);
bottomRight.configVoltageCompSaturation(12.0);
bottomRight.setInverted(false);
bottomRight.setNeutralMode(NeutralMode.Brake);
bottomRight.set(VictorSPXControlMode.PercentOutput, 0);

topLeft.configFactoryDefault();
topLeft.enableVoltageCompensation(true);
topLeft.configVoltageCompSaturation(12.0);
topLeft.setInverted(false);
topLeft.setNeutralMode(NeutralMode.Brake);
topLeft.set(VictorSPXControlMode.PercentOutput, 0);

topRight.configFactoryDefault();
topRight.enableVoltageCompensation(true);
topRight.configVoltageCompSaturation(12.0);
topRight.setInverted(false);
topRight.setNeutralMode(NeutralMode.Brake);
topRight.set(VictorSPXControlMode.PercentOutput, 0);


    }

    private void configArmCoder() {
      armCoder.configFactoryDefault();
      CANCoderUtil.setCANCoderBusUsage(armCoder, CCUsage.kSensorDataOnly);
      armCoder.configAllSettings(Robot.ctreConfigs.armCoderConfig);

    }

public void victorDrive(double power){ 
bottomLeft.set(VictorSPXControlMode.PercentOutput, power);
bottomRight.set(VictorSPXControlMode.PercentOutput, power);
topLeft.set(VictorSPXControlMode.PercentOutput, power);
topRight.set(VictorSPXControlMode.PercentOutput, power);
}

public void armUp(){
  bottomLeft.set(VictorSPXControlMode.PercentOutput, upValue);
  bottomRight.set(VictorSPXControlMode.PercentOutput, upValue);
  topLeft.set(VictorSPXControlMode.PercentOutput, upValue);
  topRight.set(VictorSPXControlMode.PercentOutput, upValue);

}
public void armDown(){
  bottomLeft.set(VictorSPXControlMode.PercentOutput, downValue);
  bottomRight.set(VictorSPXControlMode.PercentOutput, downValue);
  topLeft.set(VictorSPXControlMode.PercentOutput, downValue);
  topRight.set(VictorSPXControlMode.PercentOutput, downValue);
}
  

  @Override
  public void periodic() {
   /*  if (Slider.slidePosition > 20 && Slider.slidePosition <40 ){
      kG = 0.11;
    }
    else if (Slider.slidePosition > 40 && Slider.slidePosition <60){
      kG = 0.15;
    }
    else if (Slider.slidePosition > 60 && Slider.slidePosition <80){
      kG = 0.18;
    }
    else if (Slider.slidePosition > 80 && Slider.slidePosition <100){
      kG = 0.25;
    }
 */
  }
}
