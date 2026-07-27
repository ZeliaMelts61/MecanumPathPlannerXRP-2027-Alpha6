package first.robot.subsystems;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import java.util.Map;
import java.util.regex.Pattern;

import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.hardware.hal.SimDevice;
import org.wpilib.units.measure.Distance;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.simulation.AddressableLEDSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;

import first.robot.Constants.LEDConstants;
import first.robot.Constants.LEDConstants.LedPatterns;

import org.ejml.dense.row.misc.TransposeAlgs_CDRM;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class Leds extends SubsystemBase {
    AddressableLED m_addressableLeds = new AddressableLED(5); // don't change the channel #. If you do the bridge to the xrp won't work anymore
    AddressableLEDSim m_ledSim = new AddressableLEDSim(m_addressableLeds);
    AddressableLEDBuffer m_ledData = new AddressableLEDBuffer(LEDConstants.kNumberOfLeds);
    DigitalOutput m_ledPowerControlPin = new DigitalOutput(18);
    
    public Leds() {
        m_addressableLeds.setLength(LEDConstants.kNumberOfLeds);
        m_ledPowerControlPin.set(true);
        this.setDefaultCommand(
            getOffCommand());
        SmartDashboard.putData("Leds' Power state", m_ledPowerControlPin);
    }

    @Override public void periodic() {
        //if(m_ledPowerControlPin.get()){
            m_addressableLeds.setData(m_ledData);
        //}
    }

    public Command getOffCommand(){
        return Commands.run(() -> {
           LEDPattern.kOff.applyTo(m_ledData);
            m_addressableLeds.setData(m_ledData);
        }, this).ignoringDisable(true);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_ledData)).ignoringDisable(true);
    }

    public Command runRainbowScroll() {
        return run(() -> LedPatterns.kScrollingRainbow.applyTo(m_ledData)).ignoringDisable(true);
    }

    public Command runTransFlag(){
        return run(() -> LedPatterns.kTransFlag.applyTo(m_ledData)).ignoringDisable(true);
    }

    public void off(){
        m_ledPowerControlPin.set(false);
    }

    public void on(){
        m_ledPowerControlPin.set(true);
    }

    
}