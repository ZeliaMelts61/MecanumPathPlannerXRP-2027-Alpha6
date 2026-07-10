// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.xrp.XRPOnBoardIO;
import org.wpilib.command2.SubsystemBase;

public class Rsl extends SubsystemBase {
  /** Creates a new Rsl. */

  XRPOnBoardIO m_onboardIO;

  public Rsl(XRPOnBoardIO onboardIO) {
    this.m_onboardIO=onboardIO;
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("user button", m_onboardIO.getUserButtonPressed());
    // This method will be called once per scheduler run
  }

  public void setRSL(boolean on){
    SmartDashboard.putBoolean("user button", m_onboardIO.getUserButtonPressed());
    m_onboardIO.setLed(on);
  }

  public boolean getRSL() {
    SmartDashboard.putBoolean("user button", m_onboardIO.getUserButtonPressed());
    return m_onboardIO.getLed();
  }

  public void turnOn(){
    SmartDashboard.putBoolean("user button", m_onboardIO.getUserButtonPressed());
    setRSL(true);
  }

  public void turnOff(){
    SmartDashboard.putBoolean("user button", m_onboardIO.getUserButtonPressed());
    setRSL(false);
  }

  public void toggle(){
    SmartDashboard.putBoolean("user button", m_onboardIO.getUserButtonPressed());
    setRSL(!getRSL());
  }
}
