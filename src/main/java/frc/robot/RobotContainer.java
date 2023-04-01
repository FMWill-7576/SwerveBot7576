// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  //private final int translationAxis = XboxController.Axis.kLeftY.value;
  //private final int strafeAxis = XboxController.Axis.kLeftX.value;
  //private final int rotationAxis = XboxController.Axis.kRightX.value; 

 private final int translationAxis = 1;
 private final int strafeAxis = 0;
 private final int rotationAxis =  0; 
 private final int armAxis = XboxController.Axis.kRightY.value; 
 private final int slideAxis = XboxController.Axis.kLeftY.value;

 
 
  /* Driver Buttons */

  // private final JoystickButton zeroGyro =
  // new JoystickButton(driver, XboxController.Button.kY.value);
      private final JoystickButton robotCentric =
      new JoystickButton(driver_2, 6);    

      private final JoystickButton zeroGyro =
      new JoystickButton(driver_2, 1);

      private final JoystickButton incSpeed =
      new JoystickButton(driver_2, 5);

      private final JoystickButton decSpeed =
      new JoystickButton(driver_2, 3);

      private final JoystickButton xLock = 
      new JoystickButton(driver_2, 4);
      private final JoystickButton resetAbsolute2 = 
      new JoystickButton(driver, 8);

    //   private final JoystickButton armUp = 
    //  new JoystickButton(driver,5); 

     // private final JoystickButton armHome = 
     // new JoystickButton(driver,6);

    

      private final JoystickButton armReset = 
      new JoystickButton(driver,7);
      private final JoystickButton armDown = 
      new JoystickButton(driver,1);
      private final JoystickButton armUp = 
     new JoystickButton(driver, 4); 

    //  private final JoystickButton armDown = 
    //  new JoystickButton(driver, XboxController.Button.kA.value); 

     // private final JoystickButton pistonTest = 
     // new JoystickButton(driver, 6);


      private final JoystickButton resetAbsolute =
      new JoystickButton(driver_1,1);

      private final JoystickButton intake =
      new JoystickButton(driver,3);

      private final JoystickButton outake =
      new JoystickButton(driver,6);

      private final JoystickButton hold = 
      new JoystickButton(driver,5);

      private final JoystickButton drop =
      new JoystickButton(driver,2);

      private final JoystickButton resetSlider =
      new JoystickButton(driver,7);
      


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
 // private final VictorArm s_VictorArm = new VictorArm();
  private final Gripper s_Gripper = new Gripper();
   //private final Slider s_Slider = new Slider();
   private final Arm s_Arm = new Arm();
   //private final Vision s_Vision = new Vision();

   
   // private final Command exampleAuto = new exampleAuto(s_Swerve, s_Gripper);
    private final Command ScoreTaxiAndBalance = new ScoreTaxiAndBalance(s_Swerve, s_Gripper, s_Arm);
   // private final Command driveStraight = new driveStraight(s_Swerve, s_Gripper);
    private final Command doNothing = new doNothing(s_Swerve);
    private final Command GripOnly = new OnlyGrip(s_Swerve, s_Gripper);
    private final Command ScoreAndBalance = new ScoreAndBalance(s_Swerve, s_Gripper, s_Arm);
    private final Command ScoreAndTaxi = new ScoreAndTaxi(s_Swerve, s_Gripper, s_Arm);

    // A chooser for autonomous commands
     SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

        s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver_1.getRawAxis(translationAxis) * Swerve.speedRateSwerve,
            () -> -driver_1.getRawAxis(strafeAxis) * Swerve.speedRateSwerve,
            () -> driver_2.getRawAxis(rotationAxis) * Swerve.speedRateSwerve,
            () -> robotCentric.getAsBoolean())); 

  /*       s_VictorArm.setDefaultCommand(
          new VictorArmCommand(
            s_VictorArm,
           () -> (- driver.getRawAxis(armAxis) * 0.6) + s_VictorArm.kG)) ; 

         s_Slider.setDefaultCommand(
          new SlideCommand(
            s_Slider,
            () -> ((driver.getRawAxis(slideAxis)) * 1.0)+s_Arm.calculatedkG)) ; 
*/

         s_Gripper.setDefaultCommand(
          s_Gripper.run(() -> s_Gripper.stop()));

          s_Arm.setDefaultCommand( 
          new ArmCommand(
            s_Arm,
           () -> (-driver.getRawAxis(armAxis) * 1)+s_Arm.calculatedkG )  
          //s_Arm.run(() -> s_Arm.armDrive(1.0))


         );   
            // Add commands to the autonomous command chooser
      m_chooser.setDefaultOption("score+taxi+denge", ScoreTaxiAndBalance);
      //m_chooser.addOption("FULL RUTIN", exampleAuto);
      m_chooser.addOption("score-denge", ScoreAndBalance);
      m_chooser.addOption("nothing", doNothing);
      m_chooser.addOption("score-taxi", ScoreAndTaxi);
      m_chooser.addOption("sadece gripper", GripOnly);
        // Put the chooser on the dashboard
        SmartDashboard.putData("OTONOM", m_chooser);

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
    xLock.whileTrue(s_Swerve.run(() -> s_Swerve.xLock()));
    //armTesting.whileTrue(s_Arm.run(() -> s_Arm.armTesting()));
    resetAbsolute.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    resetAbsolute2.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
   // armUp.whileTrue(s_VictorArm.run(() -> s_VictorArm.armUp()));
   // armDown.whileTrue(s_VictorArm.run(() -> s_VictorArm.armDown()));
    //pistonTest.onTrue(s_Gripper.run(() -> s_Gripper.pistonTest()));
   // armUp.whileTrue(s_Slider.run(() -> s_Slider.armUp()));
   // armDown.whileTrue(s_Slider.run(() -> s_Slider.armDown()));
    armUp.whileTrue(s_Arm.run(() -> s_Arm.armUp()));
    armDown.whileTrue(s_Arm.run(() -> s_Arm.armDown()));
   // armHome.whileTrue(s_Arm.run(() -> s_Arm.armHome()));
    armReset.whileTrue(s_Arm.run(() -> s_Arm.armReset()));
    intake.whileTrue(s_Gripper.run(() -> s_Gripper.intake()));
    outake.whileTrue(s_Gripper.run(() -> s_Gripper.outake()));
    drop.whileTrue(s_Gripper.run(() -> s_Gripper.drop()));
    hold.toggleOnTrue(s_Gripper.run(() -> s_Gripper.hold()));
  //  resetSlider.onTrue(new InstantCommand(() -> s_Slider.resetSlider()));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new exampleAuto(s_Swerve);
    return m_chooser.getSelected();
  }
}
