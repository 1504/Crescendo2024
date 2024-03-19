// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final CANSparkMax _left_motor1;
  private final CANSparkMax _left_motor2;
  private final CANSparkMax _right_motor1;
  private final CANSparkMax _right_motor2;

  
  //encoders
  private final RelativeEncoder _left_Encoder;
  private final RelativeEncoder _right_Encoder; 

  //pid controllers
  private final PIDController _left_pid;
  private final PIDController _right_pid;
  private final PIDController _theta_pid;


  private boolean flipped = false;

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


  public Drivetrain() {
    // initializes tank motor controllers
    _left_motor1 = new CANSparkMax(DriveConstants.LEFT1, MotorType.kBrushless);
    _left_motor2 = new CANSparkMax(DriveConstants.LEFT2, MotorType.kBrushless);
    _right_motor1 = new CANSparkMax(DriveConstants.RIGHT1, MotorType.kBrushless);
    _right_motor2 = new CANSparkMax(DriveConstants.RIGHT2, MotorType.kBrushless);

    
    _right_motor1.setIdleMode(IdleMode.kBrake);
    _right_motor2.setIdleMode(IdleMode.kBrake);
    _left_motor1.setIdleMode(IdleMode.kBrake);
    _left_motor2.setIdleMode(IdleMode.kBrake);

    _right_motor1.setInverted(false);
    //_right_motor2.setInverted(true);
    _left_motor1.setInverted(true);
    //_left_motor2.setInverted(false);

    _right_motor2.follow(_right_motor1);
    _left_motor2.follow(_left_motor1);

    _left_Encoder = _left_motor1.getEncoder();
    _right_Encoder = _right_motor1.getEncoder();
    _drive = new DifferentialDrive(_left_motor1,_right_motor1);

    double p = 1.678;
    double i = 0;
    double d = 0;

    _left_pid = new PIDController(p, i, d);
    _right_pid = new PIDController(p, i, d);
    _theta_pid = new PIDController(0.01, 0, d);

    SmartDashboard.putData("Drive", _drive); 

    //odometry stuff starts
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
  _left_Encoder.getPosition(), _right_Encoder.getPosition(),
  new Pose2d(0, 0, new Rotation2d()));

    m_gyro.reset();

    //odometry stuff ends

    Pose2d m_pose = new Pose2d();
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
      _drive.feed();
      _drive.arcadeDrive(xSpd, ySpd);
  }

  public void turn( boolean direction) { //clockwise - true; counter - false
    if( direction) {
      setWheelSpeeds(-0.75,0.75);
    }
    else{
      setWheelSpeeds(0.75,-0.75);
    }
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

  public DifferentialDriveWheelPositions getCurrentState() {
    return new DifferentialDriveWheelPositions(
      getLeftVelocity(),
      getRightVelocity()
    );
  }
  
  public double getDistanceTraveled() {
    return _left_Encoder.getPosition()/BuildConstants.GR*BuildConstants.WHEEL_CIRCUMFERENCE *BuildConstants.INCHES_TO_METERS;
  }

public void setWheelSpeeds(double left, double right) {
  _left_pid.setSetpoint((left-0.19)/0.41);
  _right_pid.setSetpoint((right-0.19)/0.41);

  double left_target = MathUtil.clamp(_left_pid.calculate(getLeftVelocity()), -6, 6);
  double right_target = MathUtil.clamp(_right_pid.calculate(getRightVelocity()), -6, 6);

  _left_motor1.setVoltage(left_target);
  _right_motor1.setVoltage(right_target);
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
  
  @Override
  public void periodic() {
    m_pose = updateOdometry();
  }
}