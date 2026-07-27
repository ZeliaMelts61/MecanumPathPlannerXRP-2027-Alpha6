// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Millimeter;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Seconds;

import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.hardware.led.LEDPattern.GradientType;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final boolean runningSysid = false;
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.2;
  }
  public static class GrabberConstants {
    public static final boolean kInverted = false;
    public static final double kMinOutput = 0.0; //0.2 for box
    public static final double kMaxOutput = 1.0; //0.9 for box

    public static final double kDownAngle = 0.0; //which is technicly -40 degrees
    public static final double kUpAngle =  32.0; // technically 0 degrees
    public static final double kVerticalAngle = kUpAngle+90.0; // technically 130 degrees

    public static final double kDownPos = kDownAngle/180.0;
    public static final double kUpPos = kUpAngle/180.0;
    public static final double kVerticalPos = kVerticalAngle/180.0;
  }
  public static class RangefinderConstants {
    public static final double kLocationX = 0; //meters from the center of the robot
    public static final double kLocationY = -0.165; //meters from the center of the robot
    public static final double kAngleDegree = 180; //angle in degrees from robot forward
    public static final double kAngleRadian = org.wpilib.math.util.Units.degreesToRadians(kAngleDegree); //angle in radians from robot forward

    public static final Transform2d kRangeFinderOffset = new Transform2d(kLocationX, kLocationY, new Rotation2d(kAngleRadian));

    //
    public static final double kMinDistance = 0.1;
    public static final double kMaxDistance = 2;
    public static final double kMaxUseableAngularSpeed = 1; //radians a second
  
    
  }

  public static final class LEDConstants {
    public static final int kNumberOfLeds = 28;
    private static final int kTrueNumberOfLeds = 28;
    public static final Distance kLedStripLength = Meters.of(.92); //ish
    public static final Distance kLedSpacing = Meters.of(kLedStripLength.magnitude() / kTrueNumberOfLeds);
    
    public static final class LedPatterns {
      public static final LEDPattern kRainbow = LEDPattern.rainbow(255, 128);
      public static final LEDPattern kScrollingRainbow =
          kRainbow.scrollAtAbsoluteVelocity(MetersPerSecond.of(1), kLedSpacing);
      
      public static final LEDPattern kGradientRainbow = LEDPattern.gradient(GradientType.DISCONTINUOUS, Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.BLUE, Color.PURPLE);
      /*
        0-4 blue #5BCEFA
        5-6 pink #F5A9B8
        7-8 white #FFFFFF
        9-10 pink #F5A9B8
        11-18 blue #5BCEFA
        19-20 pink #F5A9B8
        21-22 white #FFFFFF
        23-24 pink #F5A9B8
        25-28 blue #5BCEFA
       */
      public static final Color transBlue = Color.fromString("#5BCEFA");
      public static final Color transPink = Color.fromString("#F5A9B8");
      public static final Color transWhite = Color.fromString("#FFFFFF");
      public static final LEDPattern kTransFlag = LEDPattern.steps(Map.of(
        (0/28.0), transBlue,
        (4/28.0), transPink,
        (6/28.0), transWhite,
        (8/28.0), transPink,
        (10/28.0), transBlue,
        (18/28.0), transPink,
        (20/28.0), transWhite,
        (22/28.0), transPink,
        (24/28.0), transBlue
      ));
      
      public static final LEDPattern kSolidRed = LEDPattern.solid(Color.RED);
      public static final LEDPattern kSolidOrange = LEDPattern.solid(Color.ORANGE);
      public static final LEDPattern kSolidYellow = LEDPattern.solid(Color.YELLOW);
      public static final LEDPattern kSolidGreen = LEDPattern.solid(Color.GREEN);
      public static final LEDPattern kSolidBlue = LEDPattern.solid(Color.BLUE);
      public static final LEDPattern kSolidPurple = LEDPattern.solid(Color.PURPLE);

      public static LEDPattern asBreathing(LEDPattern pattern){
        return asBreathing(pattern, Seconds.of(4));
      }
      public static LEDPattern asBreathing(LEDPattern pattern, Time period){
        return pattern.breathe(period);
      }

      public static LEDPattern asScrolling(LEDPattern pattern){
        return asScrolling(pattern, MetersPerSecond.of(1));
      }

      public static LEDPattern asScrolling(LEDPattern pattern, LinearVelocity velocity){
        return pattern.scrollAtAbsoluteVelocity(MetersPerSecond.of(1), kLedSpacing);
      }

      public static LEDPattern atBrightness(LEDPattern pattern){
        return atBrightness(pattern, Percent.of(30));
      }

      public static LEDPattern atBrightness(LEDPattern pattern, Dimensionless brightness){
        return pattern.atBrightness(brightness);
      }


    }
  }
  public static class DrivetrainConstants{
    // Mecanum wheel locations 
    public static class WheelLocationConstants{
      public static final double kFrontLeftLocationX = 0.0865; //meters
      public static final double kFrontLeftLocationY = 0.086; //meters 

      public static final double kFrontRightLocationX = 0.0865; //meters
      public static final double kFrontRightLocationY = -0.086; //meters

      public static final double kBackLeftLocationX = -0.0865; //meters
      public static final double kBackLeftLocationY = 0.086; //meters

      public static final double kBackRightLocationX = -0.0865; //meters
      public static final double kBackRightLocationY = -0.086; //meters

      public static final Translation2d kFrontLeftLocation = new Translation2d(kFrontLeftLocationX, kFrontLeftLocationY);
      public static final Translation2d kFrontRightLocation = new Translation2d(kFrontRightLocationX, kFrontRightLocationY);
      public static final Translation2d kBackLeftLocation = new Translation2d(kBackLeftLocationX, kBackLeftLocationY);
      public static final Translation2d kBackRightLocation = new Translation2d(kBackRightLocationX, kBackRightLocationY);
    }
    
    //  Odometry
    public static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    public static final double kCountsPerMotorShaftRev = 12.0;
    public static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
    public static final double kWheelDiameterMeter = Units.Meter.convertFrom(48, Millimeter); // 48 mm



    
    public static class PIDConstants {
      public static class FrontLeftPID {
        public static final double kP = 14.145; 
        public static final double kI = 0.0;
        public static final double kD = 0.0; 
      }
      public static class FrontRightPID {
        public static final double kP = 11.703;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
      public static class BackLeftPID {
        public static final double kP = 12.401; 
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
      public static class BackRightPID {
        public static final double kP = 11.899;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }

    public static class FeedforwardConstants {
      public static class TranslateFF {
        public static final double kS = -2.3188;
        public static final double kV = 17.089;
        public static final double kA = 8.8936;
      }
      public static class StrafeFF {
        public static final double kS = 3.0908;
        public static final double kV = 18.918;
        public static final double kA = 10.913;
      }
      public static class RotateFF {
        public static final double kS = 1.1537;
        public static final double kV = 17.801;
        public static final double kA = 6.122;
      }
    }

    public static class LimitSwitchConstants {
      public static final int kFrontLimitSwitchChannel = 15;
      public static final int kLeftLimitSwitchChannel = 13;
      public static final int kRightLimitSwitchChannel = 14;
      public static final int kBackLimitSwitchChannel = 12;
    }
    
    public static final double kMaxLinearXSpeedMPS = .9; //in meters per second
    public static final double kMaxLinearYSpeedMPS = .8; //in meters per second
    public static final double kMaxAngularSpeedRPS = 3.0; //in radians per second


  
  }
  
  public static class PathplannerConstants{

    public static RobotConfig config;
    static {
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
    }


    // Pid Constants

    //Translation PID constants
    public static final double kTranslationP = SmartDashboard.getNumber("kTranslationP", 5);
    public static final double kTranslationI = SmartDashboard.getNumber("kTranslationI", 0);
    public static final double kTranslationD = SmartDashboard.getNumber("kTranslationD", 0);

    public static final PIDConstants kTranslationPID = new PIDConstants(kTranslationP,kTranslationI,kTranslationD);
    // Rotation Pid Constants
    public static final double kRotationP = SmartDashboard.getNumber("kRotationP", 5);
    public static final double kRotationI = SmartDashboard.getNumber("kRotationI", 0);
    public static final double kRotationD = SmartDashboard.getNumber("kRotationD", 0);

    public static final PIDConstants kRotationPID = new PIDConstants(kTranslationP,kTranslationI,kTranslationD);

    static{
      SmartDashboard.putNumber("kTranslationP", kTranslationP);
      SmartDashboard.putNumber("kTranslationI", kTranslationI);
      SmartDashboard.putNumber("kTranslationD", kTranslationD);

      SmartDashboard.putNumber("kRotationP", kRotationP);
      SmartDashboard.putNumber("kRotationI", kRotationI);
      SmartDashboard.putNumber("kRotationD", kRotationD);

      SmartDashboard.setPersistent("kTranslationP");
      SmartDashboard.setPersistent("kTranslationI");
      SmartDashboard.setPersistent("kTranslationD");

      SmartDashboard.setPersistent("kRotationP");
      SmartDashboard.setPersistent("kRotationI");
      SmartDashboard.setPersistent("kRotationD");
    }

    
  }
}
