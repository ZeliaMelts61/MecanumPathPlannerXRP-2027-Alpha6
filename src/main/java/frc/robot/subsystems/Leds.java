package frc.robot.subsystems;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import java.util.regex.Pattern;

import org.wpilib.hardware.hal.SimDevice;
import org.wpilib.units.measure.Distance;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.simulation.AddressableLEDSim;
import org.wpilib.util.Color;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class Leds extends SubsystemBase {
    AddressableLED m_addressableLeds = new AddressableLED(0);
    AddressableLEDBuffer m_ledData = new AddressableLEDBuffer(LEDConstants.kNumberOfLeds);
    public Leds() {
        m_addressableLeds.setLength(LEDConstants.kNumberOfLeds);
        setDefaultCommand(
            Commands.idle(this)
                .withName("Off")
                .beforeStarting(Commands.runOnce(() -> {
                    LEDPattern.kOff.applyTo(m_ledData);
                    m_addressableLeds.setData(m_ledData);
                }))
        );
    }

    @Override public void periodic() {
        m_addressableLeds.setData(m_ledData);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_ledData));
    }

    public Command runRainbowScroll() {
        LEDPattern pattern = LEDPattern.rainbow(255, 128);
        return run(() -> pattern.applyTo(m_ledData));
    }

    
}