// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax _left;
  private final CANSparkMax _right;

  private final RelativeEncoder _left_encoder;
  private final RelativeEncoder _right_encoder;

  private static Arm instance = null;

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  public Arm() {
    _left = new CANSparkMax(ArmConstants.LEFT_ARM, MotorType.kBrushless);
    _right = new CANSparkMax(ArmConstants.RIGHT_ARM, MotorType.kBrushless);

    _left.setInverted(false);
    _right.setInverted(true);

    _left_encoder = _left.getEncoder();
    _right_encoder = _right.getEncoder();
  }

  public double getLeftRPM() {
    return _left_encoder.getVelocity()/9;
  }

  public double getRightRPM() {
    return _right_encoder.getVelocity()/12;
  }

  public double getLeftDistance() {
    return _left_encoder.getPosition()/9;
  }

  public double getRightDistance() {
    return _right_encoder.getPosition()/12;
  }

  public RelativeEncoder getLeftEncoder() {
    return _left_encoder;
  }

  public RelativeEncoder getRightEncoder() {
    return _right_encoder;
  }

  // Should later be set to go up a certain distance with PID controllers
  public void extendBoth(double s) {
    _left.set(s/13*4.5);
    _right.set(s/9*4.5);
  }

  public void rawLeft(double s) {
    _left.set(s/15);
  }

  public void rawRight(double s) {
    _right.set(s/9);
  }

  public void stopMotors() {
    _left.stopMotor();
    _right.stopMotor();
  }

  public void resetEncoders() {
    _left_encoder.setPosition(0);
    _right_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
  }
}