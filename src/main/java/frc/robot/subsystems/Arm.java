// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  
  // private final Compressor _compressor;
  // private final Solenoid _solomon;

  private final CANSparkMax _left;
  private final CANSparkMax _right;

  private final RelativeEncoder _left_encoder;
  private final RelativeEncoder _right_encoder;

  private boolean _up = true;
  private boolean _toggle = false;

  ShuffleboardTab w_tab = Shuffleboard.getTab("Arm");
  NetworkTableEntry positionLeft;
  NetworkTableEntry positionRight;

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

    _left.setInverted(true);
    _right.setInverted(true);

    _left_encoder = _left.getEncoder();
    _right_encoder = _right.getEncoder();
    
    // _compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    // _solomon = new Solenoid(PneumaticsModuleType.CTREPCM, WinchConstants.SOLOMON_PORT);
  }

  public void toggleSolenoid() {
    _toggle = !_toggle;
    // _solomon.set(_toggle);
  }

  public boolean direction() {
    return _up;
  }

  public void changeDirection() {
    _up = !_up;
  }

  /**
   * Gets the rpm of the left arm
   * @return the rpm of the left arm
   */
  public double getLeftRPM() {
    return _left_encoder.getVelocity();
  }
  /**
   * Gets the rpm of the right arm
   * @return the rpm of the right arm
   */
  public double getRightRPM() {
    return _right_encoder.getVelocity();
  }

  /**
   * Gets the position of the left arm
   * @return the position of the left arm
   */
  public double getLeftDistance() {
    return _left_encoder.getPosition();
  }
  /**
   * Gets the position of the right arm
   * @return the position of the right arm
   */
  public double getRightDistance() {
    return _right_encoder.getPosition();
  }

  public RelativeEncoder getLeftEncoder() {
    return _left_encoder;
  }

  public RelativeEncoder getRightEncoder() {
    return _right_encoder;
  }


  //Should later be set to go up a certain distance with PID controllers
  public void extend(double _s) {
    _left.set(_s);
    _right.set(-_s);
  }
  public void extend() {
    _left.set(-1);
    _right.set(-1);
  }

  public void contract() {
    _left.set(.4);
    _right.set(.4);
  }

  public void stopMotors() {
    _left.stopMotor();
    _right.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}