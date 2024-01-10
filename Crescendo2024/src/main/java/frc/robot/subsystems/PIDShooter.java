// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootConstants;

public class PIDShooter extends SubsystemBase {
  /** Creates a new PIDIntake. */
  private final CANSparkMax _topShooter;
  private final CANSparkMax _bottomShooter;

  private final PIDController _topPID;
  private final PIDController _bottomPID;

  private final RelativeEncoder _topEncoder;
  private final RelativeEncoder _bottomEncoder;

  public PIDShooter() {
    _topShooter = new CANSparkMax(ShootConstants.TOP_SHOOTER, MotorType.kBrushless);
    _bottomShooter = new CANSparkMax(ShootConstants.BOTTOM_SHOOTER, MotorType.kBrushless);

    _topPID = new PIDController(ShootConstants.TOP_P, ShootConstants.TOP_I, ShootConstants.TOP_D);
    _bottomPID = new PIDController(ShootConstants.BOTTOM_P, ShootConstants.BOTTOM_I, ShootConstants.BOTTOM_D);

    _topEncoder = _topShooter.getEncoder();
    _bottomEncoder = _bottomShooter.getEncoder();

    _bottomShooter.setInverted(true);
  }

  public double getBotSpeed(){
    return _bottomEncoder.getVelocity();
  }

  public double getTopSpeed(){
    return _topEncoder.getVelocity();
  }

  public void setTop(double speed){
    _topPID.setSetpoint(speed);
  }

  public void setBottom(double speed){
    _topPID.setSetpoint(speed);
  }

  public void stopShoot(){
    _topShooter.stopMotor();
    _bottomShooter.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
