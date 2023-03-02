package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
@SuppressWarnings("unused")

public class Arm extends SubsystemBase {

  public static double armSpeedRate = - 1.0;
  private CANSparkMax armMotor;
  private final ProfiledPIDController armController;
 // private RelativeEncoder integratedarmEncoder;
  private Encoder integratedArmEncoder;
  private final TrapezoidProfile.Constraints m_constraints;

  public Arm() {
      armMotor  = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushed);
      m_constraints = new TrapezoidProfile.Constraints(4.8, 3.0);
      armController = new ProfiledPIDController(Constants.Arm.armKP,Constants.Arm.armKI,Constants.Arm.armKD, m_constraints, 0.2);
;
     // integratedarmEncoder = armMotor.getEncoder();
     integratedArmEncoder = new Encoder(Constants.Arm.armEncoder,Constants.Arm.armEncoder2);
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
        integratedArmEncoder.reset();
       // integratedArmEncoder.setVelocityConversionFactor(Constants.Arm.armConversionVelocityFactor);
       // integratedArmEncoder.setPositionConversionFactor(Constants.Arm.armConversionPositionFactor);
        armController.setP(Constants.Arm.armKP);
        armController.setI(Constants.Arm.armKI);
        armController.setD(Constants.Arm.armKD);
        //armController.setFF(Constants.Arm.armKFF);
        armMotor.enableVoltageCompensation(Constants.Arm.voltageComp);
        integratedArmEncoder.setReverseDirection(true);
        integratedArmEncoder.setDistancePerPulse(Constants.Arm.armConversionPositionFactor);
        armMotor.burnFlash();
    }

      public void armTesting() {
        armController.setGoal(3.0);
        armMotor.setVoltage(armController.calculate(integratedArmEncoder.getDistance()));
      }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}