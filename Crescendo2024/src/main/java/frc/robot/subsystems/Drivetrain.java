// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.GeneralSecurityException;
import java.security.cert.TrustAnchor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  // tank motors
  private final CANSparkMax _left_motor;
  private final CANSparkMax _right_motor;
  private final RelativeEncoder _left_Encoder;
  //private GenericEntry _LEncoder;

  private boolean _turtle = false;

  private final DifferentialDrive _drive;

  //ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");

  static private Drivetrain _instance = null;

  /**
   * getInstance to provide a singleton instance of the Drivetrain subsystem
   * 
   * @return the instance of the Drivetrain subsystem
   */
  public static Drivetrain getInstance() {
    if (_instance == null) {
      _instance = new Drivetrain();
    }

    return _instance;

  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // initializes tank motor controllers
    _right_motor = new CANSparkMax(DriveConstants.RIGHT, MotorType.kBrushless);
    _left_motor = new CANSparkMax(DriveConstants.LEFT, MotorType.kBrushless);
    _left_Encoder = _right_motor.getEncoder();

    _drive = new DifferentialDrive(_left_motor, _right_motor);
    //shuffleboardInit();
  }

  // tank drive method
  public void driveTank(double xSpeed, double ySpeed) {
    // deadband the inputs
    if (!_turtle) {
      double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(ySpeed, 3);
      double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(xSpeed, 3);
      _drive.tankDrive(xSpd, ySpd);
    } else {
      double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : ySpeed / 0.5;
      double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : xSpeed / 0.5;
      _drive.tankDrive(xSpd, ySpd);
    }
  }

  public void toggleTurtle() {
    _turtle = !_turtle;
  }
  
    
  //public void shuffleboardInit() {
    //  _LEncoder = telemetry.add("Left Encoder", 0)
    //     .withPosition(0, 0)
    //      .withSize(2, 2)
    //      .withWidget(BuiltInWidgets.kTextView)
    //      .getEntry();
  //}
  
  

  //public void ShuffleboardUpdate() {
  //  _LEncoder.setDouble(_left_Encoder.getPosition());
  //}

  @Override
  public void periodic() {
    //ShuffleboardUpdate();
    // This method will be called once per scheduler run
  }
}
