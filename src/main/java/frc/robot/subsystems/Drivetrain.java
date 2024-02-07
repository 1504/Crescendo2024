// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.io.PipedInputStream;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.BuildConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class Drivetrain extends SubsystemBase {

  static private Drivetrain _instance = null;
  public static Drivetrain getInstance() {
    if (_instance == null) {
      _instance = new Drivetrain();
    }

    return _instance;
  }

  // tank motors
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

  //odometry stuff
  private final DifferentialDriveOdometry m_odometry;
  private final Gyroscope m_gyro = Gyroscope.getInstance();

  //odometry stuff ends
  DifferentialDriveKinematics kinematics =
  new DifferentialDriveKinematics(1);


  private final DifferentialDrive _drive;
  private final Limelight _limelight = Limelight.getInstance();

  private Pose2d m_pose;

  ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");

  private final AutoBuilder m_autoBuilder;


  public Drivetrain() {
    // initializes tank motor controllers
    _right_motor1 = new CANSparkMax(DriveConstants.RIGHT1, MotorType.kBrushless);
    _right_motor2 = new CANSparkMax(DriveConstants.RIGHT2, MotorType.kBrushless);
    _left_motor1 = new CANSparkMax(DriveConstants.LEFT1, MotorType.kBrushless);
    _left_motor2 = new CANSparkMax(DriveConstants.LEFT2, MotorType.kBrushless);

    _right_motor1.setInverted(false);
    //_right_motor2.setInverted(true);
    _left_motor1.setInverted(true);
    //_left_motor2.setInverted(false);

    _right_motor2.follow(_right_motor1);
    _left_motor2.follow(_left_motor1);

    _left_Encoder = _left_motor1.getEncoder();
    _right_Encoder = _right_motor1.getEncoder();

    _drive = new DifferentialDrive(_left_motor1,_right_motor1);

    double p = 1.8074;

    _left_pid = new PIDController(p, 0, 0);
    _right_pid = new PIDController(p, 0, 0);
    _theta_pid = new PIDController(0.01, 0, 0);


    SmartDashboard.putData("Drive", _drive); 

    //odometry stuff starts
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
  _left_Encoder.getPosition(), _right_Encoder.getPosition(),
  new Pose2d(0, 0, new Rotation2d()));

    m_gyro.reset();

    m_speeds = new ChassisSpeeds();

    //odometry stuff ends

    Pose2d m_pose;
    if( false) {
      //m_pose = limelight.getPose();
      m_pose = new Pose2d();
    }
    else {
      m_pose = new Pose2d();
    }
      m_autoBuilder = new AutoBuilder();

 
  AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // Current ChassisSpeeds supplier
            this::setSpeeds, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            this::flipPath,
            this // Reference to this subsystem to set requirements
    );
  }

  // tank drive method
  public void driveTank(double forwardSpeed, double rotSpeed) {
    // deadband the inputs
      double rSpd = Math.abs(rotSpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(rotSpeed, 1);
      double fSpd = Math.abs(forwardSpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(forwardSpeed, 1);
      _drive.arcadeDrive(fSpd, rSpd);
  }

  public void resetEncoders() {
    _left_Encoder.setPosition(0);
    _right_Encoder.setPosition(0);
  }

  public double getLeftVelocity() {
    return _left_Encoder.getVelocity()/BuildConstants.GR*BuildConstants.WHEEL_CIRCUMFERENCE/60 *BuildConstants.INCHES_TO_METERS;
  }

  public double getRightVelocity() {
    return _right_Encoder.getVelocity()/BuildConstants.GR*BuildConstants.WHEEL_CIRCUMFERENCE/60 *BuildConstants.INCHES_TO_METERS;

  }

  
  public PIDController getLeftPid() {
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

  public ChassisSpeeds getSpeeds() {
    double leftVelocity = getLeftVelocity();
    double rightVelocity = getRightVelocity();
    double headingVelocity = m_gyro.getRotation2d().getRadians();

    System.err.println(leftVelocity + "       " + rightVelocity + "      " + headingVelocity);
    return new ChassisSpeeds(leftVelocity, rightVelocity, headingVelocity);
}
  
  public void setSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // Left velocity
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;

    // Right velocity
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;

    //double leftVelocity = _left_pid.calculate(speeds.leftMetersPerSecond);
    //double rightVelocity = _right_pid.calculate(speeds.rightMetersPerSecond);

    setWheelSpeeds(leftVelocity, rightVelocity);
}

public void setWheelSpeeds(double right, double left) {
  _left_pid.setSetpoint(left);
  _right_pid.setSetpoint(right);
  
  _left_motor1.setVoltage(_left_pid.calculate(getLeftVelocity()));
  _right_motor1.setVoltage(_right_pid.calculate(getRightVelocity()));
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
