// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import java.util.ArrayList;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import first.robot.Constants;
import first.robot.Constants.DrivetrainConstants;
import first.robot.Constants.PathplannerConstants;
import first.robot.Utils.MecanumDriveFeedforward;
import first.robot.Utils.MecanumDriveFeedforward.MecanumDriveWheelVoltages;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.MecanumDriveKinematics;
import org.wpilib.math.kinematics.MecanumDriveOdometry;
import org.wpilib.math.kinematics.MecanumDriveWheelPositions;
import org.wpilib.math.kinematics.MecanumDriveWheelVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.EncoderSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.units.measure.Voltage;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;
import org.wpilib.util.sendable.SendableRegistry;
import org.wpilib.hardware.discrete.*;
import org.wpilib.hardware.hal.simulation.RoboRioDataJNI;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.system.Timer;
import org.wpilib.drive.MecanumDrive;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.xrp.*;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.Alliance;

public class Drivetrain extends SubsystemBase {
  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  // It also sets motors "3" and "4" to channels 2 and 3 respectively
  private final XRPMotor m_frontLeftMotor  = new XRPMotor(0);
  private final XRPMotor m_frontRightMotor = new XRPMotor(1);
  private final XRPMotor m_backLeftMotor   = new XRPMotor(2);
  private final XRPMotor m_backRightMotor  = new XRPMotor(3);
  




  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  // as well as DIO pins 8/9 and 10/11 for motors 3 and 4
  private final Encoder m_frontLeftEncoder = new Encoder(4, 5);
  private final Encoder m_frontRightEncoder = new Encoder(6, 7);
  private final Encoder m_backLeftEncoder = new Encoder(8, 9);
  private final Encoder m_backRightEncoder = new Encoder(10, 11);

  private final EncoderSim m_encodersimFL = new EncoderSim(m_frontLeftEncoder);


  // Set up the differential drive controller
  private final MecanumDrive m_mecanumDrive = new MecanumDrive(
    m_frontLeftMotor::setVoltage, m_frontRightMotor::setVoltage,
    m_backLeftMotor::setVoltage, m_backRightMotor::setVoltage
  );

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  //private final OnboardIMU m_accelerometer = new OnboardIMU();

  //Create kinematics
  private final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
      DrivetrainConstants.WheelLocationConstants.kFrontLeftLocation,
      DrivetrainConstants.WheelLocationConstants.kFrontRightLocation,
      DrivetrainConstants.WheelLocationConstants.kBackLeftLocation,
      DrivetrainConstants.WheelLocationConstants.kBackRightLocation
  );

  private ChassisVelocities m_chassisVelocities = new ChassisVelocities(0, 0, 0);

  private MecanumDriveWheelVelocities m_wheelVelocities = new MecanumDriveWheelVelocities();

  private MecanumDriveWheelPositions m_wheelPositions = new MecanumDriveWheelPositions(
      m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
      m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance()
  );


  //Create Odometry
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      m_wheelPositions
  );

  private Pose2d m_pose = new Pose2d();

  private final Field2d m_field = new Field2d();

  //pids 
  //private final PIDController m_drivePID = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  
  
  private final PIDController m_frontLeftPIDController = new PIDController(
    DrivetrainConstants.PIDConstants.FrontLeftPID.kP,
    DrivetrainConstants.PIDConstants.FrontLeftPID.kI,
    DrivetrainConstants.PIDConstants.FrontLeftPID.kD
  );

  private final PIDController m_frontRightPIDController = new PIDController(
    DrivetrainConstants.PIDConstants.FrontRightPID.kP,
    DrivetrainConstants.PIDConstants.FrontRightPID.kI,
    DrivetrainConstants.PIDConstants.FrontRightPID.kD
  );

  private final PIDController m_backLeftPIDController = new PIDController(
    DrivetrainConstants.PIDConstants.BackLeftPID.kP,
    DrivetrainConstants.PIDConstants.BackLeftPID.kI,
    DrivetrainConstants.PIDConstants.BackLeftPID.kD
  );

  private final PIDController m_backRightPIDController = new PIDController(
    DrivetrainConstants.PIDConstants.BackRightPID.kP,
    DrivetrainConstants.PIDConstants.BackRightPID.kI,
    DrivetrainConstants.PIDConstants.BackRightPID.kD
  );



  //DifferentialDriveFeedforward a = new DifferentialDriveFeedforward(getAverageDistanceInch(), getAccelZ(), getAccelY(), getAccelX());
  MecanumDriveFeedforward m_mecanumDriveFeedforward = new MecanumDriveFeedforward(
    DrivetrainConstants.FeedforwardConstants.TranslateFF.kV,
    DrivetrainConstants.FeedforwardConstants.TranslateFF.kA,
    DrivetrainConstants.FeedforwardConstants.StrafeFF.kV,
    DrivetrainConstants.FeedforwardConstants.StrafeFF.kA,
    DrivetrainConstants.FeedforwardConstants.RotateFF.kV,
    DrivetrainConstants.FeedforwardConstants.RotateFF.kA,
    m_kinematics
  );



  // stuff for displaying path in elastic
  private double m_lastTimeTargetPoseWasUpdated = Timer.getTimestamp();
  private ArrayList<Pose2d> m_pathPoses = new ArrayList<Pose2d>();
  private boolean m_hasRemovedTargetPoseAndPath = false;

  private final AnalogInput m_vinPin = new AnalogInput(3);



  // private final DigitalInput m_digital1 = new DigitalInput(1);
  // private final DigitalInput m_digital2 = new DigitalInput(2);
  // private final DigitalInput m_digital3 = new DigitalInput(3);
  // // private final DigitalInput m_digital4 = new DigitalInput(4);
  // // private final DigitalInput m_digital5 = new DigitalInput(5);
  // // private final DigitalInput m_digital6 = new DigitalInput(6);
  // // private final DigitalInput m_digital7 = new DigitalInput(7);
  // // private final DigitalInput m_digital8 = new DigitalInput(8);
  // // private final DigitalInput m_digital9 = new DigitalInput(9);
  // // private final DigitalInput m_digital10 = new DigitalInput(10);
  // // private final DigitalInput m_digital11 = new DigitalInput(11);
  // private final DigitalInput m_digital12 = new DigitalInput(12);
  // private final DigitalInput m_digital13 = new DigitalInput(13);
  // private final DigitalInput m_digital14 = new DigitalInput(14);
  // private final DigitalInput m_digital15 = new DigitalInput(15);
  // private final DigitalInput m_digital16 = new DigitalInput(16);
  // private final DigitalInput m_digital17 = new DigitalInput(17);
  // private final DigitalInput m_digital18 = new DigitalInput(18);
  // private final DigitalInput m_digital19 = new DigitalInput(19);
  // private final DigitalInput m_digital20 = new DigitalInput(20);
  // private final DigitalInput m_digital21 = new DigitalInput(21);
  // private final DigitalInput m_digital22 = new DigitalInput(22);
  // private final DigitalInput m_digital23 = new DigitalInput(23);
  // private final DigitalInput m_digital24 = new DigitalInput(24);

  // private final DigitalOutput m_digital16 = new DigitalOutput(16);
  // private final DigitalOutput m_digital17 = new DigitalOutput(17);
  // private final DigitalOutput m_digital18 = new DigitalOutput(18);

  // private final DigitalInput[] digitalInputs = new DigitalInput[] {
  //   m_digital2,
  //   m_digital3,
  //   m_digital12,
  //   m_digital13,
  //   m_digital14,
  //   m_digital15,
  //   m_digital16,
  //   m_digital17,
  //   m_digital18,
  //   m_digital19,
  //   m_digital20,
  //   m_digital22,
  //   m_digital21,
  //   m_digital23,
  //   m_digital24,
  // };
  private final DigitalInput m_frontLimitSwitchDigitalInput = new DigitalInput(DrivetrainConstants.LimitSwitchConstants.kFrontLimitSwitchChannel);
  private final DigitalInput m_leftLimitSwitchDigitalInput = new DigitalInput(DrivetrainConstants.LimitSwitchConstants.kLeftLimitSwitchChannel);
  private final DigitalInput m_rightLimitSwitchDigitalInput = new DigitalInput(DrivetrainConstants.LimitSwitchConstants.kRightLimitSwitchChannel);
  private final DigitalInput m_backLimitSwitchDigitalInput = new DigitalInput(DrivetrainConstants.LimitSwitchConstants.kBackLimitSwitchChannel);

  private final Trigger m_frontLimitSwitchTrigger = new Trigger(() -> m_frontLimitSwitchDigitalInput.get());
  private final Trigger m_leftLimitSwitchTrigger = new Trigger(() -> m_leftLimitSwitchDigitalInput.get());
  private final Trigger m_rightLimitSwitchTrigger = new Trigger(() -> m_rightLimitSwitchDigitalInput.get());
  private final Trigger m_backLimitSwitchTrigger = new Trigger(() -> m_backLimitSwitchDigitalInput.get());
  

  

  


  private double m_lastSysIdVoltage;
  private int SysidMode;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    //this sets the tolerance for the pid controllers so that they don't wiggle at low speeds
    m_frontLeftPIDController.setTolerance(0.04);
    m_frontRightPIDController.setTolerance(0.04);
    m_backLeftPIDController.setTolerance(0.04);
    m_backRightPIDController.setTolerance(0.04);


    SendableRegistry.addChild(m_mecanumDrive, m_frontLeftMotor);
    SendableRegistry.addChild(m_mecanumDrive, m_frontRightMotor);
    SendableRegistry.addChild(m_mecanumDrive, m_backLeftMotor);
    SendableRegistry.addChild(m_mecanumDrive, m_backRightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRightMotor.setInverted(true);
    m_backRightMotor.setInverted(true);

    // Use METERS as unit for encoder distances (this is different from sample XRP code)
    double distancePerPulse = (Math.PI * DrivetrainConstants.kWheelDiameterMeter) / DrivetrainConstants.kCountsPerRevolution;
    m_frontLeftEncoder.setDistancePerPulse(distancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(distancePerPulse);
    m_backLeftEncoder.setDistancePerPulse(-distancePerPulse); // I don't know why but this encoder just runs backward
    m_backRightEncoder.setDistancePerPulse(distancePerPulse);

    m_frontRightEncoder.setReverseDirection(true);
    m_backRightEncoder.setReverseDirection(true);

    resetEncoders();
    resetPose();
    resetGyro();
    createDashboardWidgets();

    // configure autobuilder for pathplanner

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotChassisVelocities, // ChassisVelocities supplier. MUST BE ROBOT RELATIVE
        (velocities, feedforwards) -> mecanumDriveRobotRelative(velocities), // Method that will drive the robot given ROBOT RELATIVE ChassisVelocities. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PathplannerConstants.kTranslationPID, // Translation PID constants
            PathplannerConstants.kRotationPID // Rotation PID constants
        ),
        PathplannerConstants.config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = MatchState.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == Alliance.RED;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  private void updateKinematics(){
    m_wheelVelocities = new MecanumDriveWheelVelocities(
        m_frontLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_backLeftEncoder.getRate(),
        m_backRightEncoder.getRate()
    );
    m_chassisVelocities = m_kinematics.toChassisVelocities(m_wheelVelocities);
  }


  private void updateOdometry(){
    //m_odometry.update(, null)

    m_wheelPositions = new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
        m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance()
    );
    m_pose = m_odometry.update(m_gyro.getRotation2d(), m_wheelPositions);

    m_field.setRobotPose(m_pose);
    SmartDashboard.putData("field", m_field);
  }


  public ChassisVelocities getFieldChassisVelocities(){
    return m_chassisVelocities.toFieldRelative(m_gyro.getRotation2d());
  }

  public ChassisVelocities getRobotChassisVelocities(){
    return m_chassisVelocities;
  }

  public void updateRioSim(){
    RoboRioSim.setVInVoltage(m_vinPin.getVoltage() * 1.55);
  }
 

  public void createDashboardWidgets(){
    SmartDashboard.putData("Mecanum", builder -> {
      builder.setSmartDashboardType("SwerveDrive");

      builder.addDoubleProperty("Front Left Angle", () -> 0.0, null);
      builder.addDoubleProperty("Front Left Velocity", () -> m_wheelVelocities.frontLeft * 10, null);

      builder.addDoubleProperty("Front Right Angle", () -> 0.0, null);
      builder.addDoubleProperty("Front Right Velocity", () -> m_wheelVelocities.frontRight * 10, null);

      builder.addDoubleProperty("Back Left Angle", () -> 0.0, null);
      builder.addDoubleProperty("Back Left Velocity", () ->  m_wheelVelocities.rearLeft * 10, null);

      builder.addDoubleProperty("Back Right Angle", () -> 0.0, null);
      builder.addDoubleProperty("Back Right Velocity", () -> m_wheelVelocities.rearRight * 10, null);

      builder.addDoubleProperty("Robot Angle", () -> m_pose.getRotation().getRadians(), null);
    });

    

    SmartDashboard.putData("Front Left Pid", m_frontLeftPIDController);
    SmartDashboard.putData("Front Right Pid", m_frontRightPIDController);
    SmartDashboard.putData("Back Left Pid", m_backLeftPIDController);
    SmartDashboard.putData("Back Right Pid", m_backRightPIDController);

    SmartDashboard.putData("Front Limit Switch", m_frontLimitSwitchDigitalInput);
    SmartDashboard.putData("Left Limit Switch", m_leftLimitSwitchDigitalInput);
    SmartDashboard.putData("Right Limit Switch", m_rightLimitSwitchDigitalInput);
    SmartDashboard.putData("Back Limit Switch", m_backLimitSwitchDigitalInput);


    // SmartDashboard.putData("digital 16", m_digital16);
    // SmartDashboard.putData("digital 17", m_digital17);
    // SmartDashboard.putData("digital 18", m_digital18);
  }

  public void updateDashboardWidgets(){
    double velocity = Math.hypot(Math.abs(m_chassisVelocities.vx),Math.abs(m_chassisVelocities.vy));
    SmartDashboard.putNumber("Linear Speed", velocity);
    SmartDashboard.putNumber("Rotational Speed", Math.abs(m_chassisVelocities.omega));
    SmartDashboard.putNumber("Match Time", MatchState.getMatchTime());
    SmartDashboard.putBoolean("Controller Connected", DriverStationBackend.isJoystickConnected(Constants.OperatorConstants.kDriverControllerPort));
    SmartDashboard.putBoolean("Robot Connected", RobotState.isDSAttached());
    SmartDashboard.putBoolean("Robot is About to explode", velocity>DrivetrainConstants.kMaxLinearXSpeedMPS);
    //RoboRioSim.getVInVoltage();
    
    double actualVoltage = m_vinPin.getVoltage();
    SmartDashboard.putNumber("Real Voltage", actualVoltage);
    SmartDashboard.putNumber("Rio Voltage", RoboRioDataJNI.getVInVoltage());

    SmartDashboard.putData("Front Left Encoder",m_frontLeftEncoder);
    SmartDashboard.putData("Front Right Encoder",m_frontRightEncoder);
    SmartDashboard.putData("Back Left Encoder",m_backLeftEncoder);
    SmartDashboard.putData("Back Right Encoder",m_backRightEncoder);
    
    SmartDashboard.putData("Drive", m_mecanumDrive);

    SmartDashboard.putBooleanArray("simEncoderBool",new Boolean[]{
      m_encodersimFL.getDirection(),
      m_encodersimFL.getInitialized(),
      m_encodersimFL.getReset(),
      m_encodersimFL.getReverseDirection()
    });

    SmartDashboard.putNumberArray("simEncoderNum",new Double[]{
      (double)m_encodersimFL.getDistance(),
      (double)m_encodersimFL.getDistancePerPulse(),
      (double)m_encodersimFL.getMaxPeriod(),
      (double)m_encodersimFL.getPeriod(),
      (double)m_encodersimFL.getRate(),
      (double)m_encodersimFL.getCount(),
      (double)m_encodersimFL.getSamplesToAverage()
    });

    

    // SmartDashboard.putData("Front Limit Switch Trigger", m_frontLimitSwitchTrigger);
    // SmartDashboard.putData("Left Limit Switch Trigger", m_leftLimitSwitchTrigger);
    // SmartDashboard.putData("Right Limit Switch Trigger", m_rightLimitSwitchTrigger);
    // SmartDashboard.putData("Back Limit Switch Trigger", m_backLimitSwitchTrigger);

    



    // for (DigitalInput digitalInput : digitalInputs) {
    //   SmartDashboard.putBoolean("Digital In " + digitalInput.getChannel(), digitalInput.get());
    // }

    // m_digital16.set(true);
    // m_digital17.set(!m_digital17.get());
    // m_digital18.set(false);
    

    //SmartDashboard.putString("digital", Arrays.toString(values.toArray()));
  }


  /* Updates the Path and TargetPose on the field widget on the dashboard */
  public void updatePathOnDashboard(){

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
    // Send to Field2d widget
      if (!poses.isEmpty()){
        m_hasRemovedTargetPoseAndPath=false;
        m_lastTimeTargetPoseWasUpdated = Timer.getTimestamp();

        m_pathPoses.addAll(poses);
        m_field.getObject("active path trajectory").setPoses(m_pathPoses);
      }
      
      
    });
  
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
      m_lastTimeTargetPoseWasUpdated = Timer.getTimestamp();
      m_field.getObject("targetPose").setPose(targetPose);
    });
    
  
    if (!(m_lastTimeTargetPoseWasUpdated+2>Timer.getTimestamp())&&!m_hasRemovedTargetPoseAndPath){
      m_hasRemovedTargetPoseAndPath=true;
      //System.out.println(m_lastTimeTargetPoseWasUpdated);
      m_field.getObject("targetPose").setPoses();
      m_field.getObject("active path trajectory").setPoses();
      m_pathPoses.clear();
    }
  }
    
  public void resetAll(){
    resetEncoders();
    resetGyro();
    resetPose(Pose2d.kZero);
  }
  
  /**
   * Drive Robot Relative using mecanum drive, uses Pids for more accurate control.
   * It will desaturate the wheel speeds automatically.
   *
   * @param xAxisSpeed Desired Speed for the robot in the X Axis (-1.0 to 1.0).
   * @param yAxisSpeed Desired Speed for the robot in the Y Axis (-1.0 to 1.0).
   * @param zAxisRotate Desired rotation Speed for the robot (-1.0 to 1.0).
   */
  public void mecanumDriveRobotRelative(double xAxisSpeed, double yAxisSpeed, double zAxisRotate) {
    ChassisVelocities Velocities = new ChassisVelocities(
      xAxisSpeed * DrivetrainConstants.kMaxLinearXSpeedMPS,
      yAxisSpeed * DrivetrainConstants.kMaxLinearYSpeedMPS,
      zAxisRotate * DrivetrainConstants.kMaxAngularSpeedRPS);
    mecanumDriveRobotRelative(Velocities, true);
  
  }

  /**
   * Drive Robot Relative using arcade drive, uses Pids for more accurate control.
   * It will desaturate the wheel speeds automatically.
   *
   * @param xAxisSpeed Desired Speed for the robot in the X Axis (-1.0 to 1.0).
   * @param zAxisRotate Desired rotation Speed for the robot (-1.0 to 1.0).
   */
  public void arcadeDrive(double xAxisSpeed, double zAxisRotate) {
    // if(allEqualZero(xAxisSpeed, zAxisRotate)){
    //   stop();
    //   return;
    // }
    
    ChassisVelocities Velocities = new ChassisVelocities(
        xAxisSpeed * DrivetrainConstants.kMaxLinearXSpeedMPS,
        0,
        zAxisRotate * DrivetrainConstants.kMaxAngularSpeedRPS);
    mecanumDriveRobotRelative(Velocities, true);
  }
  

  /**
   * Drive Robot Relative using mecanum drive, Uses pids for more accurate control.
   * It will desaturate the wheel speeds automatically.
   * @param velocities The ChassisVelocities you want the robot to move at. 
   */
  public void mecanumDriveRobotRelative(ChassisVelocities velocities, boolean fromJoystick) {
    setWheelVelocities(velocities, fromJoystick);
  }

  

  public void mecanumDriveRobotRelative(ChassisVelocities velocities) {
    mecanumDriveRobotRelative(velocities, false); 
  }

  
  
  /**
   * Drive Field Relative using mecanum drive, Uses pids for more accurate control.
   * It will desaturate the wheel speeds automatically
   * @param xAxisSpeed Desired Speed for the robot in the field X Axis (-1.0 to 1.0).
   * @param yAxisSpeed Desired Speed for the robot in the field Y Axis (-1.0 to 1.0).
   * @param zAxisRotate Desired rotation Speed for the robot (-1.0 to 1.0). 
   * @param gyroAngle The rotation 2d of the robot.
   */
  public void mecanumDriveFieldRelative(double xAxisSpeed, double yAxisSpeed, double zAxisRotate, Rotation2d gyroAngle) {
    // if(allEqualZero(xAxisSpeed,yAxisSpeed,zAxisRotate)){
    //   stop();
    //   return;
    // }
    ChassisVelocities velocities = 
      new ChassisVelocities(
        xAxisSpeed * DrivetrainConstants.kMaxLinearXSpeedMPS,
        yAxisSpeed * DrivetrainConstants.kMaxLinearYSpeedMPS,
        zAxisRotate * DrivetrainConstants.kMaxAngularSpeedRPS
      ).toFieldRelative(gyroAngle);
    
    
    mecanumDriveRobotRelative(velocities, true);
  }
  

  /**
   * Drive Field Relative using mecanum drive.
   * It uses the built in gyro to calculate robot rotation.
   * Uses pids for more accurate control.
   * It will desaturate the wheel speeds automatically.
   * @param xAxisSpeed Desired Speed for the robot in the field X Axis (-1.0 to 1.0).
   * @param yAxisSpeed Desired Speed for the robot in the field Y Axis (-1.0 to 1.0).
   * @param zAxisRotate Desired rotation Speed for the robot (-1.0 to 1.0). 
   */
  public void mecanumDriveFieldRelative(double xAxisSpeed, double yAxisSpeed, double zAxisRotate) {
    mecanumDriveFieldRelative(xAxisSpeed, yAxisSpeed, zAxisRotate, m_gyro.getRotation2d());
  }

  public void mecanumDriveNoPid(double xAxisSpeed, double yAxisSpeed, double zAxisRotate){
    m_mecanumDrive.driveCartesian(xAxisSpeed, yAxisSpeed, zAxisRotate);
  }
    

  /**
   * Sets the mecanum wheel speeds using closed-loop velocity control.
   * 
   * <p>This method:
   * <ul>
   *   <li>Desaturates wheel speeds to the drivetrain maximum velocity.</li>
   *   <li>Uses PID controllers to correct wheel velocity error.</li>
   *   <li>Uses feedforward to improve velocity tracking accuracy.</li>
   *   <li>Combines PID and feedforward outputs for final motor commands.</li>
   *   <li>Clamps motor outputs to the valid range of -1.0 to 1.0.</li>
   * </ul>
   * 
   * <p>The supplied wheel velocities are expected to be in meters per second.
   * 
   * @param targetVelocities Desired wheel velocities for each mecanum wheel.
  */
  private void setWheelVelocities(ChassisVelocities targetChassisVelocities, boolean fromJoystick) {
    MecanumDriveWheelVoltages wheelVoltagesFF;
    if(fromJoystick) {
      targetChassisVelocities = removeMovementsFromLimitSwitches(targetChassisVelocities);
      wheelVoltagesFF = m_mecanumDriveFeedforward.calculateWithoutkA(m_chassisVelocities, targetChassisVelocities, 0.02);
    } else {
      wheelVoltagesFF = m_mecanumDriveFeedforward.calculate(m_chassisVelocities, targetChassisVelocities, 0.02);
    }
    MecanumDriveWheelVelocities targetVelocities = m_kinematics.toWheelVelocities(targetChassisVelocities);
    targetVelocities=targetVelocities.desaturate(DrivetrainConstants.kMaxLinearXSpeedMPS);

    double frontLeftPIDOut =
        m_frontLeftPIDController.calculate(
            m_wheelVelocities.frontLeft,
            targetVelocities.frontLeft);
    double frontRightPIDOut = 
        m_frontRightPIDController.calculate(
                m_wheelVelocities.frontRight,
                targetVelocities.frontRight);
    double backLeftPIDOut =
        m_backLeftPIDController.calculate(
            m_wheelVelocities.rearLeft,
            targetVelocities.rearLeft);
    double backRightPIDOut =
        m_backRightPIDController.calculate(
            m_wheelVelocities.rearRight,
            targetVelocities.rearRight);
    

    double frontLeftOutput = wheelVoltagesFF.frontLeft() + frontLeftPIDOut;
    double frontRightOutput = wheelVoltagesFF.frontRight() + frontRightPIDOut;
    double backLeftOutput = wheelVoltagesFF.rearLeft() + backLeftPIDOut;
    double backRightOutput = wheelVoltagesFF.rearRight() + backRightPIDOut;

    double maxVolt = Math.max(
      Math.max(Math.abs(frontLeftOutput), Math.abs(frontRightOutput)),
      Math.max(Math.abs(backLeftOutput), Math.abs(backRightOutput))
    );


    if (maxVolt > 12.0) {
      double scale = 12.0 / maxVolt;
      frontLeftOutput *= scale;
      frontRightOutput *= scale;
      backLeftOutput *= scale;
      backRightOutput *= scale;
    }
    
    m_frontLeftMotor.setVoltage(frontLeftOutput);
    m_frontRightMotor.setVoltage(frontRightOutput);
    m_backLeftMotor.setVoltage(backLeftOutput);
    m_backRightMotor.setVoltage(backRightOutput);
    m_mecanumDrive.feed(); // oooooh free food............
  }
  

  // private MecanumDriveWheelVelocities removeMovementsFromLimitSwitches(MecanumDriveWheelVelocities targetVelocities) {
  //   if(m_frontLimitSwitchDigitalInput.get()){
  //     targetVelocities.
  //   }
  // }

  private ChassisVelocities removeMovementsFromLimitSwitches(ChassisVelocities targetChassisVelocities) {
    if(!(m_frontLimitSwitchDigitalInput.get() && m_backLimitSwitchDigitalInput.get())){
      if(m_frontLimitSwitchDigitalInput.get() && targetChassisVelocities.vx > 0){
        targetChassisVelocities.vx = 0;
      } else if(m_backLimitSwitchDigitalInput.get() && targetChassisVelocities.vx < 0){
        targetChassisVelocities.vx = 0;
      }
    }
    
    if(!(m_leftLimitSwitchDigitalInput.get() && m_rightLimitSwitchDigitalInput.get())){
      if(m_leftLimitSwitchDigitalInput.get() && targetChassisVelocities.vy > 0){
        targetChassisVelocities.vy = 0;
      }
      else if(m_rightLimitSwitchDigitalInput.get() && targetChassisVelocities.vy < 0){
        targetChassisVelocities.vy = 0;
      }
    }
    return targetChassisVelocities;
  }

  /**
   * Stops the drivetrain by setting all motor powers to 0.
   */
  public void stop(){
    mecanumDriveRobotRelative(new ChassisVelocities());
    m_frontLeftMotor.setVoltage(0);
    m_frontRightMotor.setVoltage(0);
    m_backLeftMotor.setVoltage(0);
    m_backRightMotor.setVoltage(0);

    SmartDashboard.putBoolean("Stopped", true);
    m_mecanumDrive.feed();
  }
    

  /**
   * Gets the current robot pose
   * @return The current pose2d of the robot.
   */
  public Pose2d getPose(){
    return m_pose;
  }

  /**
   * Gets the field from the Drive system
   * @return The Field
   */
  public Field2d getField(){
    return m_field;
  }

  /**
   * Resets the Robot's pose to the supplied pose
   * @param newPose the pose the robot will get set to
   */
  public void resetPose(Pose2d newPose) {
    //resetEncoders();
    //updateOdometry();

    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        m_wheelPositions,
        newPose);

    m_pose = newPose;
  }

  /**
   * Resets the Robot's pose to Pose2d.kZero
   */
  public void resetPose() {
    resetPose(Pose2d.kZero);
  }  

  public ChassisVelocities getRobotRelativeSpeeds(){
    return m_chassisVelocities;
  }

  /**
   * Resets all the Drive Encoder distances to zero. 
   * Resets all of the Drive Encoder current counts to zero.
   */
  public void resetEncoders() {
    m_frontLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_backLeftEncoder.reset();
    m_backRightEncoder.reset();
  }

  /**
   * Gets the current count of the Front Left Encoder.
   * Returns the current count on the Front Left Encoder.
   * This method compensates for the decoding type.
   * @return Current count from the Front Left Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int getFrontLeftEncoderCount() {
    return m_frontLeftEncoder.get();
  }

  /**
   * Gets the current count of the Front Right Encoder.
   * Returns the current count on the Front Right Encoder.
   * This method compensates for the decoding type.
   * @return Current count from the Front Right Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int getFrontRightEncoderCount() {
    return m_frontRightEncoder.get();
  }

  /**
   * Gets the current count of the Back Left Encoder.
   * Returns the current count on the Back Left Encoder.
   * This method compensates for the decoding type.
   * @return Current count from the Back Left Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int getBackLeftEncoderCount() {
    return m_backLeftEncoder.get();
  }

  /**
   * Gets the current count of the Back Right Encoder.
   * Returns the current count on the Back Right Encoder.
   * This method compensates for the decoding type.
   * @return Current count from the Back Right Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int getBackRightEncoderCount() {
    return m_backRightEncoder.get();
  }

  
  /**
  * Gets the distance the Front Left wheel has driven (in meters) since the last reset.
  * @return The distance in meters driven since the last reset
  */
  public double getFrontLeftDistanceMeters() {
    return m_frontLeftEncoder.getDistance();
  }

  /**
  * Gets the distance the Front Right wheel has driven (in meters) since the last reset.
  * @return The distance in meters driven since the last reset
  */
  public double getFrontRightDistanceMeters() {
    return m_frontRightEncoder.getDistance();
  }

  /**
  * Gets the distance the Back Left wheel has driven (in meters) since the last reset.
  * @return The distance in meters driven since the last reset
  */
  public double getBackLeftDistanceMeters() {
    return m_backLeftEncoder.getDistance();
  }

  /**
  * Gets the distance the Back Right wheel has driven (in meters) since the last reset.
  * @return The distance in meters driven since the last reset
  */
  public double getBackRightDistanceMeters() {
    return m_backRightEncoder.getDistance();
  }

  /**
  * Gets the average distance the left wheels have driven (in meters) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getAverageLeftDistanceMeters() {
    return (getFrontLeftDistanceMeters() + getBackLeftDistanceMeters()) / 2;
  }

  /**
  * Gets the average distance the left wheels have driven (in inches) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getLeftDistanceInch() {
    return Units.metersToInches(getAverageLeftDistanceMeters());
  }

  /**
  * Gets the average distance the right wheels have driven (in inches) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getRightDistanceInch() {
    return Units.metersToInches(getAverageRightDistanceMeters());
  }

  /**
  * Gets the average distance the right wheels have driven (in meters) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getAverageRightDistanceMeters() {
    return (getFrontRightDistanceMeters() + getBackRightDistanceMeters()) / 2;
  }

  /**
  * Gets the average distance the robot has driven (in meters) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getAverageDistanceMeters() {
    return (getAverageLeftDistanceMeters() + getAverageRightDistanceMeters()) / 2.0;
  }

  /**
  * Gets the average distance the robot has driven (in inches) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getAverageDistanceInch() {
    return Units.metersToInches(getAverageDistanceMeters());
  }

  // /**
  //  * The acceleration in the X-axis.
  //  *
  //  * @return The acceleration of the XRP along the X-axis in Gs
  //  */
  // public double getAccelX() {
  //   return m_accelerometer.getX();
  // }

  // /**
  //  * The acceleration in the Y-axis.
  //  *
  //  * @return The acceleration of the XRP along the Y-axis in Gs
  //  */
  // public double getAccelY() {
  //   return m_accelerometer.getY();
  // }

  // /**
  //  * The acceleration in the Z-axis.
  //  *
  //  * @return The acceleration of the XRP along the Z-axis in Gs
  //  */
  // public double getAccelZ() {
  //   return m_accelerometer.getZ();
  // }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public Command resetGyroCommand(){
    return new InstantCommand(() -> resetGyro()).andThen(new InstantCommand(() -> System.out.println("Gyro reset")));
  }

  public Trigger getFrontLimitSwitchTrigger(){
    return m_frontLimitSwitchTrigger;
  }

  public Trigger getLeftLimitSwitchTrigger(){
    return m_leftLimitSwitchTrigger;
  }
  
  public Trigger getRightLimitSwitchTrigger(){
    return m_rightLimitSwitchTrigger;
  }
  
  public Trigger getBackLimitSwitchTrigger(){
    return m_backLimitSwitchTrigger;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateKinematics();
    updateOdometry();
    updateRioSim();
    updatePathOnDashboard();
    updateDashboardWidgets();
  }

  private static boolean allEqualZero(double... nums){
    for(double num : nums){
      if(num!=0.0){
        return false;
      }
    }
    return true;
  }

  private static double max(double... nums){
    double max = Double.MAX_VALUE;
    for(double num : nums){
      max=Math.max(max, num);
    }
    return max;
  }

  private final SysIdRoutine.Config m_sysIdConfig = new SysIdRoutine.Config(
    Volts.of(2).per(Second),         // Default ramp rate (1V/s)
    Volts.of(7),  // Step voltage for dynamic tests (Reduce if space is small)
    Seconds.of(6),         // Default timeout (10 seconds)
    null          // Default log consumer (Logs directly to DataLogManager/WPILog)
  );

  // 2. Define the Translation (Forward/Backward) Routine
  private final SysIdRoutine m_translationRoutine = new SysIdRoutine(
      m_sysIdConfig,
      new SysIdRoutine.Mechanism(
          (Voltage volts) -> {
            // Drive all 4 motors forward equally
            SysidMode=1;
            m_lastSysIdVoltage = volts.in(Volts);
            double v = volts.in(Volts);
            m_frontLeftMotor.setVoltage(v);
            m_frontRightMotor.setVoltage(v);
            m_backLeftMotor.setVoltage(v);
            m_backRightMotor.setVoltage(v);
          },
          this::logDriveData, // Feed encoder and voltage data into SysId
          this
      )
  );

  // 3. Define the Strafing (Sideways) Routine
  private final SysIdRoutine m_strafeRoutine = new SysIdRoutine(
      m_sysIdConfig,
      new SysIdRoutine.Mechanism(
          (Voltage volts) -> {
            // Apply diagonal voltages to force the mecanum wheels to slide sideways
            SysidMode=2;
            m_lastSysIdVoltage = volts.in(Volts);
            double v = volts.in(Volts);
            m_frontLeftMotor.setVoltage(v);
            m_frontRightMotor.setVoltage(-v);
            m_backLeftMotor.setVoltage(-v);
            m_backRightMotor.setVoltage(v);
          },
          this::logDriveData, // Uses the exact same logger format
          this
      )
  );

  private final SysIdRoutine m_rotationRoutine = new SysIdRoutine(
      m_sysIdConfig,
      new SysIdRoutine.Mechanism(
          (Voltage volts) -> {
            // Apply diagonal voltages to force the mecanum wheels to slide sideways
            SysidMode=3;
            m_lastSysIdVoltage = volts.in(Volts);
            double v = volts.in(Volts);
            m_frontLeftMotor.setVoltage(-v);
            m_frontRightMotor.setVoltage(v);
            m_backLeftMotor.setVoltage(-v);
            m_backRightMotor.setVoltage(v);
          },
          this::logDriveData, // Uses the exact same logger format
          this
      )
  );

  private void logDriveData(SysIdRoutineLog log) {
    switch (SysidMode) {
      case 1 ->
          log.motor("drive")
              .voltage(Volts.of(m_lastSysIdVoltage))
              .linearPosition(Meters.of(getPose().getX()))
              .linearVelocity(MetersPerSecond.of(getRobotRelativeSpeeds().vx));

      case 2 ->
          log.motor("drive")
              .voltage(Volts.of(m_lastSysIdVoltage))
              .linearPosition(Meters.of(getPose().getY()))
              .linearVelocity(MetersPerSecond.of(getRobotRelativeSpeeds().vy));

      case 3 ->
          log.motor("drive")
              .voltage(Volts.of(m_lastSysIdVoltage))
              .angularPosition(Radians.of(getPose().getRotation().getRadians()))
              .angularVelocity(RadiansPerSecond.of(getRobotRelativeSpeeds().omega));
    } 
  }

  public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
    return m_translationRoutine.quasistatic(direction).beforeStarting(() -> resetPose());
  }

  public Command sysIdDynamicTranslation(SysIdRoutine.Direction direction) {
    return m_translationRoutine.dynamic(direction).beforeStarting(() -> resetPose());
  }

  public Command sysIdQuasistaticStrafe(SysIdRoutine.Direction direction) {
    return m_strafeRoutine.quasistatic(direction).beforeStarting(() -> resetPose());
  }

  public Command sysIdDynamicStrafe(SysIdRoutine.Direction direction) {
    return m_strafeRoutine.dynamic(direction).beforeStarting(() -> resetPose());
  }

  public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
    return m_rotationRoutine.quasistatic(direction).beforeStarting(() -> resetPose());
  }

  public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
    return m_rotationRoutine.dynamic(direction).beforeStarting(() -> resetPose());
  }
  
}
