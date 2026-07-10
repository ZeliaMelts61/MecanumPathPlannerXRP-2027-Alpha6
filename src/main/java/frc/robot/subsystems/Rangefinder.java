// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Distance;
import org.wpilib.xrp.XRPRangefinder;
import org.wpilib.command2.SubsystemBase;

public class Rangefinder extends SubsystemBase {
  /** Creates a new Rangefinder. */
  XRPRangefinder m_rangefinder;
  public Rangefinder() {
    m_rangefinder=new XRPRangefinder();
  }

  public double getDistanceMeters(){
    return m_rangefinder.getDistance();
  }

  public double getDistanceInches(){
    return getDistanceAsUnit(Inches).magnitude();
  }

  public Distance getDistanceMetersMeasure() {
    return Meters.of(getDistanceMeters());
  }

  public Distance getDistanceAsUnit(DistanceUnit unit){
    //org.wpilib.units.Units.Meters;
    
    return unit.of(unit.convertFrom(getDistanceMeters(), Meters));
  }


  @Override
  public void periodic() {}
}
