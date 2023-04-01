package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private static double kS = 0.02;
  private static double kG = 0.50;
  private static double kV = 0.93;
  public double calculatedkG;
  private ArmFeedforward armFeedforward;
  public static double armSpeedRate =  1.0;
  private CANSparkMax armMotor;
  private CANSparkMax armMotor2;
  private final SparkMaxPIDController armController;
  private final SparkMaxPIDController armController2;
  private RelativeEncoder integratedArmEncoder;
  private RelativeEncoder integratedArmEncoder2;
  //private final TrapezoidProfile.Constraints m_constraints;

  public Arm() {
      armMotor  = new CANSparkMax(27, MotorType.kBrushless);
      armMotor2  = new CANSparkMax(26, MotorType.kBrushless);
      //m_constraints = new TrapezoidProfile.Constraints(4.8, 3.0);
      armController = armMotor.getPIDController();
      armController2 = armMotor2.getPIDController();
      armFeedforward = new ArmFeedforward(kS, kG, kV);
;
     // integratedarmEncoder = armMotor.getEncoder();
     integratedArmEncoder = armMotor.getEncoder();
     integratedArmEncoder2 = armMotor2.getEncoder();
     armMotorConfig();

  }
 



      public void armMotorConfig() { 
        armMotor.restoreFactoryDefaults();
        armMotor2.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor2, Usage.kPositionOnly);
        armMotor.setSmartCurrentLimit(40);
        armMotor2.setSmartCurrentLimit(40);
        armMotor.setInverted(true);
        armMotor.setIdleMode(Constants.ArmConstants.armNeutralMode);
        armMotor2.setInverted(true);
        armMotor2.setIdleMode(Constants.ArmConstants.armNeutralMode);
         
        integratedArmEncoder.setPositionConversionFactor(180.0/24.33); 
        integratedArmEncoder.setPosition(-45.0);
       
        integratedArmEncoder2.setPositionConversionFactor(180.0/24.33); 
        integratedArmEncoder2.setPosition(-45.0); 
        armController.setP(Constants.ArmConstants.armKP);
        armController.setI(Constants.ArmConstants.armKI);
        armController.setD(Constants.ArmConstants.armKD);
        armController2.setP(Constants.ArmConstants.armKP);
        armController2.setI(Constants.ArmConstants.armKI);
        armController2.setD(Constants.ArmConstants.armKD);
        //armController.setFF(Constants.ArmConstants.armKFF);
        armMotor.enableVoltageCompensation(Constants.ArmConstants.voltageComp);
        armMotor2.enableVoltageCompensation(12.0);
        //integratedArmEncoder.setReverseDirection(false); // bu
        //integratedArmEncoder.setDistancePerPulse(Constants.ArmConstants.armConversionPositionFactor); //bu
        armMotor.burnFlash();
       //armMotor2.follow(armMotor);
        armMotor2.burnFlash();
    }

      public void armSet(Rotation2d angle) {
       
        armController.setReference(
        angle.getDegrees(),
        ControlType.kPosition,
        0,
        armFeedforward.calculate(angle.getRadians(), 0));

        armController2.setReference(
          angle.getDegrees(),
          ControlType.kPosition,
          0,
          armFeedforward.calculate(angle.getRadians(), 0));

      }
     

     public void armUp(){
     // armSet(Rotation2d.fromDegrees(150.0));
     armDrive(0.265);
     }

     public void armCone(){
       armSet(Rotation2d.fromDegrees(29.0));}

       public void armCube(){
        armSet(Rotation2d.fromDegrees(140.0));}

     public void armDrive(double armPercentage){
  armMotor.set(armPercentage);
  armMotor2.set(armPercentage);
      }    

     public void armHome(){
      armSet(Rotation2d.fromDegrees(-40.0));
     }

     public void armDown(){
      //armSet(Rotation2d.fromDegrees(200.0));
      armDrive(-0.265);
     }


     public void armReset(){
      integratedArmEncoder.setPosition(-45.0);
      integratedArmEncoder2.setPosition(-45.0);
     }

    @Override
    public void periodic() {
      if (integratedArmEncoder.getPosition() > 85.0 && integratedArmEncoder.getPosition() < 95.0 )
      {calculatedkG = 0.0; }
      else if (integratedArmEncoder.getPosition()> 95.0){
        calculatedkG = -0.075;
      }
      else {
      calculatedkG = 0.065 ; }
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("arm encoder" , integratedArmEncoder.getPosition());
      //SmartDashboard.putNumber("arm distance" , integratedArmEncoder.getDistance());
    }
}