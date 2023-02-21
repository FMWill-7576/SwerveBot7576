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

public class Elevator extends SubsystemBase {

  public static double speedRate = 1.0;
  private CANSparkMax elevatorMotor;
  private final SparkMaxPIDController elevatorController;
  private RelativeEncoder integratedElevatorEncoder;

  public Elevator() {
     elevatorMotor  = new CANSparkMax(Constants.elevator.elevatorMotorID, MotorType.kBrushless);
     elevatorController = elevatorMotor.getPIDController();
     integratedElevatorEncoder = elevatorMotor.getEncoder();
     elevatorMotorConfig();

  }
 

public void elevatorDrive(double elevatorPercentage){
  elevatorMotor.set(elevatorPercentage);
      }    

      public void elevatorMotorConfig() { 
        elevatorMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(elevatorMotor, Usage.kAll);
        elevatorMotor.setSmartCurrentLimit(Constants.elevator.elevatorContinuousCurrentLimit);
        elevatorMotor.setInverted(Constants.elevator.elevatorInvert);
        elevatorMotor.setIdleMode(Constants.elevator.elevatorNeutralMode);
        integratedElevatorEncoder.setVelocityConversionFactor(Constants.elevator.elevatorConversionVelocityFactor);
        integratedElevatorEncoder.setPositionConversionFactor(Constants.elevator.elevatorConversionPositionFactor);
        elevatorController.setP(Constants.elevator.elevatorKP);
        elevatorController.setI(Constants.elevator.elevatorKI);
        elevatorController.setD(Constants.elevator.elevatorKD);
        elevatorController.setFF(Constants.elevator.elevatorKFF);
        elevatorMotor.enableVoltageCompensation(Constants.elevator.voltageComp);
        elevatorMotor.burnFlash();
        integratedElevatorEncoder.setPosition(0.0);
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}


