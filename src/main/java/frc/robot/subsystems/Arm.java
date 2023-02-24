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

public class Arm extends SubsystemBase {

  public static double armSpeedRate = 0.5;
  private CANSparkMax armMotor;
  private final SparkMaxPIDController armController;
 // private RelativeEncoder integratedarmEncoder;

  public Arm() {
     armMotor  = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushed);
     armController = armMotor.getPIDController();
     // integratedarmEncoder = armMotor.getEncoder();
     armMotorConfig();

  }
 

public void armDrive(double armPercentage){
  armMotor.set(armPercentage);
      }    

      public void armMotorConfig() { 
        armMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kAll);
        armMotor.setSmartCurrentLimit(Constants.Arm.armContinuousCurrentLimit);
        armMotor.setInverted(Constants.Arm.armInvert);
        armMotor.setIdleMode(Constants.Arm.armNeutralMode);
        //integratedarmEncoder.setVelocityConversionFactor(Constants.Arm.armConversionVelocityFactor);
        //integratedarmEncoder.setPositionConversionFactor(Constants.Arm.armConversionPositionFactor);
        armController.setP(Constants.Arm.armKP);
        armController.setI(Constants.Arm.armKI);
        armController.setD(Constants.Arm.armKD);
        armController.setFF(Constants.Arm.armKFF);
        armMotor.enableVoltageCompensation(Constants.Arm.voltageComp);
        armMotor.burnFlash();
        // integratedarmEncoder.setPosition(0.0);
    }

      public void armTesting() {
        armController.setReference(30.0, CANSparkMax.ControlType.kPosition);
      }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}