// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.system.Timer;
import org.wpilib.xrp.XRPOnBoardIO;

import first.robot.Robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.smartdashboard.SmartDashboard;

public class Rsl extends SubsystemBase {
  /** Creates a new Rsl. */

  XRPOnBoardIO m_onboardIO;

  public Rsl(XRPOnBoardIO onboardIO) {
    SmartDashboard.putData("rsl subsystem", this);
    this.m_onboardIO=onboardIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("rsl led", m_onboardIO.getLed());

  }

  public void setRSL(boolean on){
    m_onboardIO.setLed(on);
  }

  public boolean getRSL() {
    return m_onboardIO.getLed();
  }

  public void turnOn(){
    setRSL(true);
  }

  public void turnOff(){
    setRSL(false);
  }

  public void toggle(){
    setRSL(!getRSL());
  }


  public Command getRslCommand(){
    Command rslCommand = new Command() {
      Timer changeState = new Timer();
      boolean runWhileDisabled = true;
      @Override
      public void initialize() {
        changeState.restart();
        changeState.start();
        turnOn();
        runWhileDisabled = false;
      }
      @Override
      public void execute() {
        SmartDashboard.putNumber("rslTimer", changeState.get());
        if(Robot.isEnabled()){
          if(changeState.advanceIfElapsed(0.4)){
            toggle();
          }

        }
      }
      @Override
      public boolean isFinished() {
        return false;
      }
      @Override
      public void end(boolean interrupted) {
        if (!DriverStationBackend.isDSAttached()) {
          turnOff();
        } else {
          turnOn();
        }
      }
      @Override
      public boolean runsWhenDisabled() {
        return false;
      }
      
    };
    rslCommand.addRequirements(this);
    return rslCommand;
  }
}
