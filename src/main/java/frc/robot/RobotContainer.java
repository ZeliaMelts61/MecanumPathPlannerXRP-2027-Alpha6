// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.LEDConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.xrp.XRPOnBoardIO;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.PrintCommand;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.GenericHID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();
  private final Grabber m_grabber = new Grabber();
  //private final Leds m_leds = new Leds();
  private final Rsl m_rsl = new Rsl(m_onboardIO);

  // Assumes a gamepad plugged into channel 0
  //private final Joystick m_controller = new Joystick(0);
  //public final SwappableController m_controller = new SwappableController(0, this::configureButtonBindings);
  public final CommandGamepad m_controller = new CommandGamepad(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = AutoBuilder.buildAutoChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureAutoBindings();
    // Configure the button bindings
    
    m_rsl.setDefaultCommand(new RslCommand(m_rsl));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * org.wpilib.driverstation.Joystick} or {@link XboxController}), and then passing it to a {@link
   * org.wpilib.command2.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    m_drivetrain.setDefaultCommand(getMecanumDriveCommand().ignoringDisable(false));

    /* 
    if (OperatorConstants.runningSysid){
      sysidMappings();
      return;
    }
    */
    m_controller.leftBumper()
      .whileTrue(new InstantCommand(() -> m_drivetrain.resetGyro()))
      .onChange(new InstantCommand(() -> m_drivetrain.resetGyro()));
    m_grabber.setAngle(80);
  
    Trigger userButton = new Trigger(m_onboardIO::getUserButtonPressed);
    userButton
        .onTrue(new PrintCommand("USER Button Pressed"))
        .onFalse(new PrintCommand("USER Button Released"))
        .onChange(new InstantCommand(() -> SmartDashboard.putBoolean("user button", m_onboardIO.getUserButtonPressed())));

    // m_controller.faceUp() //eg the Y button on an xbox controller
    //     .onTrue(new InstantCommand (() -> m_drivetrain.resetAll()).ignoringDisable(true));
    // m_controller.faceDown() //eg the A button on an xbox controller
    //     .onTrue(m_grabber.vertical());
    // m_controller.faceRight() //eg the B button on an xbox controller
    // .onTrue(m_grabber.max());


    m_controller.northFace() //eg the Y button on an xbox controller
        .onTrue(new InstantCommand (() -> m_drivetrain.resetAll()).ignoringDisable(true));
    m_controller.southFace() //eg the A button on an xbox controller
        .onTrue(m_grabber.vertical());
    m_controller.eastFace() //eg the B button on an xbox controller
        .onTrue(m_grabber.max());

    m_controller.leftTrigger() 
        .onTrue(m_grabber.toggle());

    // m_controller.dpadUp()
    //   .whileTrue(m_leds.runPattern(LEDConstants.LedPatterns.asBreathing(LEDConstants.LedPatterns.kGradientRainbow)));
    // m_controller.dpadDown()
    //   .whileTrue(m_leds.runRainbowScroll());

    
  }
  
  /*
  public void sysidMappings(){
    //translate Quasistatic forward -> x && left bumper
    m_controller.x().and(m_controller.leftBumper()).whileTrue(m_drivetrain.sysIdQuasistaticTranslation(SysIdRoutine.Direction.kForward));

    //translate Quasistatic backward -> b && left bumper
    m_controller.b().and(m_controller.leftBumper()).whileTrue(m_drivetrain.sysIdQuasistaticTranslation(SysIdRoutine.Direction.kReverse));

    //translate Dynamic forward -> x && right bumper
    m_controller.x().and(m_controller.rightBumper()).whileTrue(m_drivetrain.sysIdDynamicTranslation(SysIdRoutine.Direction.kForward));

    //translate Dynamic forward -> b && right bumper
    m_controller.b().and(m_controller.rightBumper()).whileTrue(m_drivetrain.sysIdDynamicTranslation(SysIdRoutine.Direction.kReverse));



    //strafe Quasistatic left -> y && left bumper
    m_controller.y().and(m_controller.leftBumper()).whileTrue(m_drivetrain.sysIdQuasistaticStrafe(SysIdRoutine.Direction.kReverse));
    
    //strafe Quasistatic right -> a && left bumper
    m_controller.a().and(m_controller.leftBumper()).whileTrue(m_drivetrain.sysIdQuasistaticStrafe(SysIdRoutine.Direction.kForward));

    //strafe Dynamic left -> y && right bumper
    m_controller.y().and(m_controller.rightBumper()).whileTrue(m_drivetrain.sysIdDynamicStrafe(SysIdRoutine.Direction.kReverse));
    
    //strafe Dynamic right -> a && right bumper
    m_controller.a().and(m_controller.rightBumper()).whileTrue(m_drivetrain.sysIdDynamicStrafe(SysIdRoutine.Direction.kForward));


    
    //rotate Quasistatic counter clockwise -> leftTrigger && left bumper
    m_controller.leftTrigger().and(m_controller.leftBumper()).whileTrue(m_drivetrain.sysIdQuasistaticRotation(SysIdRoutine.Direction.kForward));

    //rotate Quasistatic clockwise -> rightTrigger && left bumper
    m_controller.rightTrigger().and(m_controller.leftBumper()).whileTrue(m_drivetrain.sysIdQuasistaticRotation(SysIdRoutine.Direction.kReverse));

    //rotate Dynamic counter clockwise -> leftTrigger && right bumper
    m_controller.leftTrigger().and(m_controller.rightBumper()).whileTrue(m_drivetrain.sysIdDynamicRotation(SysIdRoutine.Direction.kForward));

    //rotate Dynamic clockwise -> rightTrigger && right bumper
    m_controller.rightTrigger().and(m_controller.rightBumper()).whileTrue(m_drivetrain.sysIdDynamicRotation(SysIdRoutine.Direction.kReverse));
  
  
  }
  */

  private void configureAutoBindings(){
    NamedCommands.registerCommand("Grabber up", m_grabber.up());
    NamedCommands.registerCommand("Grabber down", m_grabber.down());
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    //m_chooser.addOption("Pathplanner Test Auto", new PathPlannerAuto("New Auto"));
    //m_chooser.addOption("curve", new PathPlannerAuto("curve"));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getMecanumDriveCommand() {
    return new MecanumDrive(
        m_drivetrain, () -> -m_controller.getLeftY(), () -> -m_controller.getLeftX(), () -> -m_controller.getRightX(), () -> m_controller.leftBumper().getAsBoolean());
  }
}
