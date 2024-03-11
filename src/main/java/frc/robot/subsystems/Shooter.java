// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.ShootConstants;

public class Shooter extends SubsystemBase {

  private static Shooter _instance = null;

  private final CANSparkMax _rightShooter;
  private final CANSparkMax _leftShooter;

  private final RelativeEncoder _rightEncoder;
  private final RelativeEncoder _leftEncoder;

  public static Shooter getInstance() {
    if (_instance == null) {
      _instance = new Shooter();
    }
    return _instance;
  }

  public Shooter() {
    _rightShooter = new CANSparkMax(ShootConstants.RIGHT_SHOOTER, MotorType.kBrushless);
    _leftShooter = new CANSparkMax(ShootConstants.LEFT_SHOOTER, MotorType.kBrushless);

    _rightEncoder = _rightShooter.getEncoder();
    _leftEncoder = _leftShooter.getEncoder();

    _leftShooter.setInverted(true);
  }

  public double getLeftSpeed() {
    return _leftEncoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60
        * BuildConstants.INCHES_TO_METERS;
  }

  public double getRightSpeed() {
    return _rightEncoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60
        * BuildConstants.INCHES_TO_METERS;
  }

  public void stopShoot() {
    _rightShooter.stopMotor();
    _leftShooter.stopMotor();
  }

  public void shoot() {
    _rightShooter.set(-1);
    _leftShooter.set(-1);
  }

  @Override
  public void periodic() {
  }
}