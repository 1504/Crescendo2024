// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  // tank motors
  private final CANSparkMax _left_motor;
  private final CANSparkMax _right_motor;

  private final DifferentialDrive _drive;

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
 
    _drive = new DifferentialDrive(_left_motor, _right_motor);
  }

  // tank drive method
  public void driveTank(double xSpeed, double ySpeed) {
    xSpeed *= -1;
    // deadband the inputs
    double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(ySpeed, 3);
    double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(xSpeed, 3);
    _drive.tankDrive(xSpd, ySpd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
