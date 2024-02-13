// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
<<<<<<< HEAD
=======
/*import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
*/
>>>>>>> shotoer
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootConstants;

public class PIDShooter extends SubsystemBase {
  /** Creates a new PIDIntake. */
  private final CANSparkMax _rightShooter;
  private final CANSparkMax _leftShooter;

  private final PIDController _rightPID;
  private final PIDController _leftPID;

  private final RelativeEncoder _rightEncoder;
  private final RelativeEncoder _leftEncoder;

  public PIDShooter() {
    _rightShooter = new CANSparkMax(ShootConstants.RIGHT_SHOOTER, MotorType.kBrushless);
    _leftShooter = new CANSparkMax(ShootConstants.LEFT_SHOOTER, MotorType.kBrushless);

    _rightPID = new PIDController(ShootConstants.RIGHT_P, ShootConstants.RIGHT_I, ShootConstants.RIGHT_D);
    _leftPID = new PIDController(ShootConstants.LEFT_P, ShootConstants.LEFT_I, ShootConstants.LEFT_D);

    _rightEncoder = _rightShooter.getEncoder();
    _leftEncoder = _leftShooter.getEncoder();

    _leftShooter.setInverted(true);
  }

  public double getLeftSpeed(){
    return _leftEncoder.getVelocity();
  }

  public double getRightSpeed(){
    return _rightEncoder.getVelocity();
  }

  public void setRight(double speed){
    _rightPID.setSetpoint(speed);
  }

  public void setLeft(double speed){
    _rightPID.setSetpoint(speed);
  }

  public void stopShoot(){
    _rightShooter.stopMotor();
    _leftShooter.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}