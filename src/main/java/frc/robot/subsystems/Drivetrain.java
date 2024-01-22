// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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
  
  private final RelativeEncoder _left_Encoder;
  private final RelativeEncoder _right_Encoder; 

  private boolean _turtle = false;

  private final DifferentialDrive _drive;
  private final Limelight _limelight = Limelight.getInstance();

  private Pose2d m_pose;

  ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");



  public Drivetrain() {
    // initializes tank motor controllers
    _right_motor1 = new CANSparkMax(DriveConstants.RIGHT1, MotorType.kBrushless);
    _right_motor2 = new CANSparkMax(DriveConstants.RIGHT2, MotorType.kBrushless);
    _left_motor1 = new CANSparkMax(DriveConstants.LEFT1, MotorType.kBrushless);
    _left_motor2 = new CANSparkMax(DriveConstants.LEFT2, MotorType.kBrushless);

    _right_motor1.setInverted(false);
    //_right_motor2.setInverted(true);
    _left_motor1.setInverted(false);
    //_left_motor2.setInverted(false);

    _right_motor2.follow(_right_motor1);
    _left_motor2.follow(_left_motor1);

    _left_Encoder = _left_motor1.getEncoder();
    _right_Encoder = _right_motor1.getEncoder();

    _drive = new DifferentialDrive(_left_motor1,_right_motor1);

    SmartDashboard.putData("Drive", _drive); 

    Pose2d m_pose;
    if( false) {
      //m_pose = limelight.getPose();
      m_pose = new Pose2d();
    }
    else {
      m_pose = new Pose2d();
    }
  }

  // tank drive method
  public void driveTank(double xSpeed, double ySpeed) {
    // deadband the inputs
    if (!_turtle) {
      double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(ySpeed, 1);
      double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(xSpeed, 1);
      _drive.arcadeDrive(xSpd, ySpd);
    } else {
      double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : ySpeed / 0.5;
      double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : xSpeed / 0.5;
      _drive.arcadeDrive(xSpd, ySpd);
    }
  }

  public void toggleTurtle() {
    _turtle = !_turtle;
  }

  public void resetEncoders() {
    _left_Encoder.setPosition(0);
    _right_Encoder.setPosition(0);
  }

  public double getLeftVelocity() {
    return _left_Encoder.getVelocity();

    //return _front_left_encoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  public double getRightVelocity() {
    return _right_Encoder.getVelocity();
  }

  //public Pose2d getPose() {
    //return _odometry.getPoseMeters();
  //}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
