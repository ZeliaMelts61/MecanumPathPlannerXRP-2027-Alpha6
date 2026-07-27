// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import org.wpilib.math.util.MathUtil;
import org.wpilib.driverstation.Joystick;
import org.wpilib.smartdashboard.SmartDashboard;

import first.robot.Constants.OperatorConstants;
import first.robot.Constants.OperatorConstants.*;
import first.robot.subsystems.Drivetrain;

import org.wpilib.command2.Command;
import java.util.function.Supplier;

public class MecanumDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_yaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final Supplier<Boolean> m_robotRelativeSupplier;

  /**
   * Creates a new MecanumDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public MecanumDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> yaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      Supplier<Boolean> robotRelativeSupplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_yaxisSpeedSupplier = yaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    m_robotRelativeSupplier = robotRelativeSupplier;
    SmartDashboard.putNumber("Robot Speed", 1);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    double xSpeed = MathUtil.applyDeadband(m_xaxisSpeedSupplier.get(), OperatorConstants.kDeadband)* SmartDashboard.getNumber("Robot Speed", 1);
    double ySpeed = MathUtil.applyDeadband(m_yaxisSpeedSupplier.get(), OperatorConstants.kDeadband)* SmartDashboard.getNumber("Robot Speed", 1);
    double rotSpeed = MathUtil.applyDeadband(m_zaxisRotateSupplier.get(), OperatorConstants.kDeadband) * SmartDashboard.getNumber("Robot Speed", 1);
    boolean driveRobotRelative = m_robotRelativeSupplier.get();
    SmartDashboard.putNumber("JoystickXSpeed", xSpeed);
    SmartDashboard.putNumber("JoystickYSpeed", ySpeed);
    SmartDashboard.putNumber("JoystickZSpeed", rotSpeed);
    //System.out.println(driveRobotRelative);
    //m_drivetrain.mecanumDriveRobotRelative(xSpeed, ySpeed, rotSpeed);
    if(driveRobotRelative){
      m_drivetrain.mecanumDriveRobotRelative(xSpeed, ySpeed, rotSpeed);
    } else {
      m_drivetrain.mecanumDriveFieldRelative(xSpeed, ySpeed, rotSpeed);
    }
    //m_drivetrain.mecanumDriveNoPid(xSpeed, ySpeed, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
