package frc.lib.config;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;

public final class CTREConfigs {
  public CANCoderConfiguration swerveCanCoderConfig;
  public CANCoderConfiguration armCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANCoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  


  armCoderConfig = new CANCoderConfiguration();
  /* Arm CANcoder Configuraration */
  armCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
  armCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
  armCoderConfig.initializationStrategy =
      SensorInitializationStrategy.BootToZero;
  armCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
