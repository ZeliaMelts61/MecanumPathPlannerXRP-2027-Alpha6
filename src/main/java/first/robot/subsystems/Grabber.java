// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.xrp.XRPServo;

import first.robot.Constants.GrabberConstants;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.discrete.AnalogInput;

public class Grabber extends SubsystemBase {
  private final XRPServo m_servoLeft;
  private final XRPServo m_servoRight;
  private boolean inverted = false;
  /** Creates a new Arm. */
  public Grabber() {
    // Device number 4 maps to the physical Servo 1 port on the XRP
    m_servoLeft = new XRPServo(4);
    m_servoRight = new XRPServo(5);
    inverted = GrabberConstants.kInverted;
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  /**
   * Set the current angle of the arm (0 - 180 degrees).
   *
   * @param angleDeg Desired arm angle in degrees
   */
  public void setAngle(double angleDeg) {
    angleDeg=clamp(angleDeg, 0, 180);
    setPosition(angleDeg / 180.0);    
  }
  public void setPosition(double amount){
    amount=clamp(amount, 0, 1);
    amount=scale((inverted ? 1.0-amount : amount));
    m_servoLeft.setPosition(amount);
    m_servoRight.setPosition(1-amount);

  }
  public double getPosition(){
    double avgPos = (m_servoLeft.getPosition()+(1.0-m_servoRight.getPosition()))/2;
    double truePos = (inverted ? 1.0-avgPos : avgPos);
    return unscale(truePos);
  }

  public double getAngle(){
    return getPosition()*180;
  }

  public void setToDown(){
    setPosition(GrabberConstants.kDownPos);
  }

  public void setToUp(){
    setPosition(GrabberConstants.kUpPos);
  }

  public void setToVertical(){
    setPosition(GrabberConstants.kVerticalPos);
  }

  public Command down(){
    return Commands.runOnce(()->setToDown(), this);
  }

  public Command up(){
    return Commands.runOnce(()->setToUp(), this);
  }

  public Command vertical(){
    return Commands.runOnce(()->setToVertical(), this);
  }

  public Command max() {
    return setPositionCommand(1);
  }

  public Command min() {
    return setPositionCommand(0);
  }

  public Command toggle(){
    return Commands.runOnce(()-> {
      if(getAngle()<=GrabberConstants.kUpAngle-5){setToUp();}
      else {setToDown();}
    }, this);
    
  }

  public Command setAngleCommand(double angle){
    return Commands.runOnce(()->setAngle(angle), this);
  }

  public Command setPositionCommand(double position){
    return Commands.runOnce(()->setPosition(position), this);
  }

  private static double scale(double value) {
    double outMin=GrabberConstants.kMinOutput;
    double outMax=GrabberConstants.kMaxOutput;
    value = clamp(value, 0.0, 1.0);
    return (value - 0.0) * (outMax - outMin) / (1.0 - 0.0) + outMin;
  }

  private static double unscale(double value) {
    double outMin = GrabberConstants.kMinOutput;
    double outMax = GrabberConstants.kMaxOutput;
    value = clamp(value, outMin, outMax);
    return (value - outMin) / (outMax - outMin);
  }

  private static double clamp(double value, double min, double max){
    if(value>=max){
      return max;
    }
    if(value<=min){
      return min;
    }
    return value;
  }



}
