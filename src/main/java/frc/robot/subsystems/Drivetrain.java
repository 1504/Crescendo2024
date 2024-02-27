// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  static private Drivetrain _instance = null;
  
  public static Drivetrain getInstance() {
    if (_instance == null) {
      _instance = new Drivetrain();
    }

    return _instance;
  }

  //private final CANSparkMax _left_motor1;
  //private final CANSparkMax _left_motor2;
  //private final CANSparkMax _right_motor1;
  //private final CANSparkMax _right_motor2;

  private final CANSparkMax _right_motor1 = new CANSparkMax(DriveConstants.RIGHT1, MotorType.kBrushless);
  private final CANSparkMax _right_motor2 = new CANSparkMax(DriveConstants.RIGHT2, MotorType.kBrushless);
  private final CANSparkMax _left_motor1 = new CANSparkMax(DriveConstants.LEFT1, MotorType.kBrushless);
  private final CANSparkMax _left_motor2 = new CANSparkMax(DriveConstants.LEFT2, MotorType.kBrushless);
  
  private final RelativeEncoder _left_Encoder = _left_motor1.getEncoder();
  private final RelativeEncoder _right_Encoder = _right_motor1.getEncoder();
  //encoders
  //private final RelativeEncoder _left_Encoder;
  //private final RelativeEncoder _right_Encoder; 

  //pid controllers
  private final PIDController _left_pid;
  private final PIDController _right_pid;
  private final PIDController _theta_pid;


  private boolean flipped = false;


  private final SimpleMotorFeedforward feedForward;

  //odometry stuff
  private final DifferentialDriveOdometry m_odometry;
  private final Gyroscope m_gyro = Gyroscope.getInstance();

  //odometry stuff ends
  DifferentialDriveKinematics kinematics =
  new DifferentialDriveKinematics(24*Constants.BuildConstants.INCHES_TO_METERS);


  private DifferentialDrive _drive;
  private final Limelight _limelight = Limelight.getInstance();

  private Pose2d m_pose;

  ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");



  public static final Voltage Volts = BaseUnits.Voltage;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  public Drivetrain() {
    // initializes tank motor controllers
    //_right_motor1 = new CANSparkMax(DriveConstants.RIGHT1, MotorType.kBrushless);
    //_right_motor2 = new CANSparkMax(DriveConstants.RIGHT2, MotorType.kBrushless);
    //_left_motor1 = new CANSparkMax(DriveConstants.LEFT1, MotorType.kBrushless);
    //_left_motor2 = new CANSparkMax(DriveConstants.LEFT2, MotorType.kBrushless);

    /* 
    _right_motor1.setIdleMode(IdleMode.kBrake);
    _right_motor2.setIdleMode(IdleMode.kBrake);
    _left_motor1.setIdleMode(IdleMode.kBrake);
    _left_motor2.setIdleMode(IdleMode.kBrake);
    */

    _right_motor1.setInverted(false);
    //_right_motor2.setInverted(true);
    _left_motor1.setInverted(true);
    //_left_motor2.setInverted(false);

    _right_motor2.follow(_right_motor1);
    _left_motor2.follow(_left_motor1);

    //_left_Encoder = _left_motor1.getEncoder();
    //_right_Encoder = _right_motor1.getEncoder();


    _drive = new DifferentialDrive(_left_motor1,_right_motor1);

    double p = 1; //left 6.1261, right 7.5226
    double i = 0;
    double d = 0; //left 0.20986, right 0.1059775

    _left_pid = new PIDController(p, i, d);
    _right_pid = new PIDController(p, i, d);
    _theta_pid = new PIDController(0.01, 0, d);

    feedForward = new SimpleMotorFeedforward(0.3, 0); //0.3


    SmartDashboard.putData("Drive", _drive); 

    //odometry stuff starts
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
  _left_Encoder.getPosition(), _right_Encoder.getPosition(),
  new Pose2d(0, 0, new Rotation2d()));

    m_gyro.reset();

    //odometry stuff ends

    Pose2d m_pose = new Pose2d();
  
 
  AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // Current ChassisSpeeds supplier
            this::setSpeeds, //setSpeeds, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            this::flipPath,
            this // Reference to this subsystem to set requirements
    );
  }

  public Command stopDrivetrain() {
    return run(() -> {
      driveTank(0, 0);
    });
  }

  // tank drive method
  public void driveTank(double xSpeed, double ySpeed) {
    // deadband the inputs
      double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(ySpeed, 1);
      double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(xSpeed, 1);
      _drive.arcadeDrive(xSpd, ySpd);
  }

  public void drivePID(double velocity) {
    setWheelSpeeds(velocity, velocity);

  }
  public boolean getFlipped() {
    return flipped;
  }

  public void resetEncoders() {
    _left_Encoder.setPosition(0);
    _right_Encoder.setPosition(0);
  }

  public RelativeEncoder getLeftEncoder() {
    return _left_Encoder;
  }

  public RelativeEncoder getRightEncoder() {
    return _right_Encoder;
  }

  public double getLeftVelocity() {
    return _left_Encoder.getVelocity()/BuildConstants.GR*BuildConstants.WHEEL_CIRCUMFERENCE/60 *BuildConstants.INCHES_TO_METERS;
  }

  public double getRightVelocity() {
    return _right_Encoder.getVelocity()/BuildConstants.GR*BuildConstants.WHEEL_CIRCUMFERENCE/60 *BuildConstants.INCHES_TO_METERS;
  }

  public void stopMotors() {
    _left_motor1.set(0);
    _right_motor1.set(0);
  }
  
  public PIDController getLeftPID() {
    return _left_pid;
  }

  public PIDController getRightPID() {
    return _right_pid;
  }

  public PIDController getAnglePID() {
    return _theta_pid;
  }
  
  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */

  public DifferentialDriveWheelPositions getCurrentState() {
    return new DifferentialDriveWheelPositions(
      getLeftVelocity(),
      getRightVelocity()
    );
  }
  
  public double getDistanceTraveled() {
    return _left_Encoder.getPosition()/BuildConstants.GR*BuildConstants.WHEEL_CIRCUMFERENCE *BuildConstants.INCHES_TO_METERS;
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.2);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(targetSpeeds);
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;
    setWheelSpeeds(leftVelocity, rightVelocity);
  }

  public ChassisSpeeds getSpeeds() {
    double leftVelocity = getLeftVelocity();
    double rightVelocity = getRightVelocity();
    double headingVelocity = m_gyro.getRotation2d().getRadians();
    System.err.println(leftVelocity + "       " + rightVelocity + "      " + headingVelocity);

    return new ChassisSpeeds(leftVelocity, rightVelocity, headingVelocity);
    //return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()));
}
  
  public void setSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // Left velocity
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    // Right velocity
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;

    setWheelSpeeds(leftVelocity, rightVelocity);
}

public void setWheelSpeeds(double left, double right) {
  _left_pid.setSetpoint((left-0.19)/0.41);
  _right_pid.setSetpoint((right-0.19)/0.41);

  //_left_pid.setSetpoint(left);
  //_right_pid.setSetpoint(right);

  double left_target = MathUtil.clamp(_left_pid.calculate(getLeftVelocity()), -6, 6);
  double right_target = MathUtil.clamp(_right_pid.calculate(getRightVelocity()), -6, 6);

  //System.err.println(feedForward.calculate(left_target));
  //System.err.println(_left_pid.calculate(getLeftVelocity()));
  //_left_motor1.setVoltage((_left_pid.calculate(getLeftVelocity()) + feedForward.calculate(left_target))/0.6);
  //_right_motor1.setVoltage((_right_pid.calculate(getRightVelocity()) + feedForward.calculate(right_target))/.6);

  _left_motor1.setVoltage(left_target);
  _right_motor1.setVoltage(right_target);


  //_left_motor1.setVoltage(feedForward.calculate(left_target) + left_target);
  //_right_motor1.setVoltage(feedForward.calculate(right_target) + right_target);
}

public void tuneKS(double voltage) {
  _left_motor1.setVoltage(voltage);
  _right_motor1.setVoltage(voltage);
}

  public Boolean flipPath() {
    return false;
  }

  public Pose2d updateOdometry() {
    return m_odometry.update(m_gyro.getRotation2d(), getCurrentState());
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentState(), pose);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                _left_motor1.setVoltage(volts.in(Volts));
                _right_motor1.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            _left_motor1.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(_left_Encoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(_left_Encoder.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            _right_motor1.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(_right_Encoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(_right_Encoder.getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

    /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  
  @Override
  public void periodic() {
    m_pose = updateOdometry();
  }
}