// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
   private final Joystick driver = new Joystick(0);
  private final Joystick driver_1 = new Joystick(1);
  private final Joystick driver_2 = new Joystick(2);
  

  /* Drive Controls */
  private final int slideAxis = XboxController.Axis.kLeftY.value;
  //private final int strafeAxis = XboxController.Axis.kLeftX.value;
  //private final int rotationAxis = XboxController.Axis.kRightX.value; 

 private final int translationAxis = 1;
 private final int strafeAxis = 0;
 private final int rotationAxis =  0; 

  /* Driver Buttons */

  // private final JoystickButton zeroGyro =
  // new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver_2, 5);    
  private final JoystickButton zeroGyro =
      new JoystickButton(driver_2, 1);
      private final JoystickButton incSpeed =
      new JoystickButton(driver_2, 2);
      private final JoystickButton decSpeed =
      new JoystickButton(driver_2, 3);


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Slider s_Slider = new Slider();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

        s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver_1.getRawAxis(translationAxis) * Swerve.speedRate,
            () -> -driver_1.getRawAxis(strafeAxis) * Swerve.speedRate,
            () -> -driver_2.getRawAxis(rotationAxis) * Swerve.speedRate,
            () -> robotCentric.getAsBoolean())); 
        
         s_Slider.setDefaultCommand(
          new SliderCommand(
            s_Slider,
            () -> driver.getRawAxis(slideAxis)* Slider.speedRate)) ;

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    incSpeed.whileTrue(new InstantCommand(() -> s_Swerve.incSpeed()));
    decSpeed.whileTrue(new InstantCommand(() -> s_Swerve.decSpeed()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
