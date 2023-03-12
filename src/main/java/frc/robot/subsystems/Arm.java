package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public static double armSpeedRate =  1.0;
  private CANSparkMax armMotor;
  private final ProfiledPIDController armController;
  private Encoder integratedArmEncoder;
  private final TrapezoidProfile.Constraints m_constraints;

  public Arm() {
      armMotor  = new CANSparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushed);
      m_constraints = new TrapezoidProfile.Constraints(4.8, 3.0);
      armController = new ProfiledPIDController(Constants.ArmConstants.armKP,Constants.ArmConstants.armKI,Constants.ArmConstants.armKD, m_constraints, 0.2);
;
     // integratedarmEncoder = armMotor.getEncoder();
     integratedArmEncoder = new Encoder(0,1);
     armMotorConfig();

  }
 

public void armDrive(double armPercentage){
  armMotor.set(armPercentage);
      }    

      public void armMotorConfig() { 
        armMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kAll);
        armMotor.setSmartCurrentLimit(Constants.ArmConstants.armContinuousCurrentLimit);
        armMotor.setInverted(Constants.ArmConstants.armInvert);
        armMotor.setIdleMode(Constants.ArmConstants.armNeutralMode);
        // integratedArmEncoder.reset(); bu
       // integratedArmEncoder.setVelocityConversionFactor(Constants.ArmConstants.armConversionVelocityFactor);
       // integratedArmEncoder.setPositionConversionFactor(Constants.ArmConstants.armConversionPositionFactor); 
        armController.setP(Constants.ArmConstants.armKP);
        armController.setI(Constants.ArmConstants.armKI);
        armController.setD(Constants.ArmConstants.armKD);
        //armController.setFF(Constants.ArmConstants.armKFF);
        armMotor.enableVoltageCompensation(Constants.ArmConstants.voltageComp);
        //integratedArmEncoder.setReverseDirection(false); // bu
        //integratedArmEncoder.setDistancePerPulse(Constants.ArmConstants.armConversionPositionFactor); //bu
        armMotor.burnFlash();
    }

      public void armTesting() {
        armController.setGoal(20.0);
        armMotor.setVoltage(armController.calculate(integratedArmEncoder.getDistance()));
      }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      //SmartDashboard.putNumber("arm encoder" , integratedArmEncoder.get());
      //SmartDashboard.putNumber("arm distance" , integratedArmEncoder.getDistance());
    }
}