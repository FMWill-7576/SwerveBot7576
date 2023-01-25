package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Slider extends SubsystemBase {

  public static double speedRate = 1.0;
  private CANSparkMax slideMotor;
  private final SparkMaxPIDController slideController;
  private RelativeEncoder integratedSlideEncoder;

  public Slider() {
     slideMotor  = new CANSparkMax(Constants.Slider.slideMotorID, MotorType.kBrushless);
     slideController = slideMotor.getPIDController();
     integratedSlideEncoder = slideMotor.getEncoder();
     slideMotorConfig();

  }
 

public void slide(double slidePercentage){
  slideMotor.set(slidePercentage);
      }    

      public void slideMotorConfig() { 
        slideMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(slideMotor, Usage.kPositionOnly);
        slideMotor.setSmartCurrentLimit(Constants.Slider.slideContinuousCurrentLimit);
        slideMotor.setInverted(Constants.Slider.slideInvert);
        slideMotor.setIdleMode(Constants.Slider.slideNeutralMode);
        //integratedSlideEncoder.setVelocityConversionFactor(Constants.Slider.slideConversionVelocityFactor);
        //integratedSlideEncoder.setPositionConversionFactor(Constants.Slider.slideConversionPositionFactor);
        slideController.setP(Constants.Slider.slideKP);
        slideController.setI(Constants.Slider.slideKI);
        slideController.setD(Constants.Slider.slideKD);
        slideController.setFF(Constants.Slider.slideKFF);
        slideMotor.enableVoltageCompensation(Constants.Slider.voltageComp);
        slideMotor.burnFlash();
        integratedSlideEncoder.setPosition(0.0);
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}


