// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private final CANSparkMax m_motor = new CANSparkMax(IntakeConstants.intakePort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final double MAXSPEED = 0.5;

  private boolean auto = false;


  private static GroundIntake instance = null;

  PIDController intake_pid; 

  private GroundIntake() {
    //intake_pid = new PIDController(IntakeConstants.kP, 0, 0);
    m_motor.setInverted(true); 
    //m_motor.setIdleMode(IdleMode.kBrake);
    // TODO: ADD SOFT BREAK IN INTAKE USING INFARED SENSOR
  }

  public static GroundIntake getInstance() {
    if (instance == null) {
      instance = new GroundIntake();
    }
    return instance;
  }

  /*public PIDController getPID() {
    return intake_pid;
  }*/

  public void roll(double speed) {
    m_motor.set(speed);
  }

  public void rawIntake() {
    if (!auto) {
      m_motor.set(MAXSPEED);
    }
  }

  /**
   * Reverse spin the intake without using PID
   */
  public void rawReverse() {
    if (!auto) {
      m_motor.set(-MAXSPEED);
    }
  }

  /**
   * Stops the motor
   */
  public void stopMotor() {
    m_motor.set(0);
  }

  /**
   * Toggles the auto mode
   */
  public void toggleAuto() {
    auto = !auto;
  }

  /**
   * Sets the auto mode
   *
   * @param a the auto mode to set to
   */
  public void setAuto(boolean a) {
    auto = a;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
