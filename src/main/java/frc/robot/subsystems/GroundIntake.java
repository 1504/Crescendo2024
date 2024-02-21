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
  private final CANSparkMax motor_1 = new CANSparkMax(IntakeConstants.intakePort_1, MotorType.kBrushless); //motor_1 - intake
  private final CANSparkMax motor_2 = new CANSparkMax(IntakeConstants.intakePort_2, MotorType.kBrushless); // flips
  private final RelativeEncoder m_encoder_1 = motor_1.getEncoder();
  private final RelativeEncoder m_encoder_2 = motor_2.getEncoder();

  private boolean auto = false;

  private static GroundIntake instance = null;

  private double curr_pos = 0;

  private boolean down = false;

  private GroundIntake() {
    
  }

  public static GroundIntake getInstance() {
    if (instance == null) {
      instance = new GroundIntake();
    }
    return instance;
  }

  public void roll(){
    motor_1.set(IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void outRoll() {
    motor_1.set(-IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void rawIntake() {
    if (!auto) {
      motor_1.set(IntakeConstants.MAX_INTAKE_SPEED);
    }
  }

  /**
   * Stops the motor
   */
  public void stopMotor1() {
    motor_1.set(0);
  }
  public void stopMotor2() {
    motor_2.set(0);
  }

  public void rawFlipUp(){
    motor_2.set(IntakeConstants.MAX_FLIP_SPEED);
  }

  public void rawFlipDown() {
    motor_2.set(-IntakeConstants.MAX_FLIP_SPEED);
  }

  

  public RelativeEncoder getFlipEncoder() {
    return m_encoder_2;
  }


  /**
   * Toggles the auto mode
  public void toggleAuto() {
    auto = !auto;
  }

  /**
   * Sets the auto mode
   *
   * @param a the auto mode to set to
  
  public void setAuto(boolean a) {
    auto = a;
  }
  */

  /* 
  public void setSetpoint(double setpoint){
    intake_pid.setSetpoint(setpoint);
    curr_pos = setpoint;
  }

  public void addSetpoint(double amt){
    intake_pid.setSetpoint(curr_pos + amt);
  }
  */

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double val = m_encoder.getPosition();
    //motor_1.set(val);
  }
}
