// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakePort_1, MotorType.kBrushless); //motor_1 - intake
  private final CANSparkMax flipperMotor = new CANSparkMax(IntakeConstants.intakePort_2, MotorType.kBrushless); // flips
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private final RelativeEncoder flipperEncoder = flipperMotor.getEncoder();

  private boolean auto = false;

  private static GroundIntake instance = null;

  private GroundIntake() {
    flipperMotor.setIdleMode(IdleMode.kBrake); 
  }

  public static GroundIntake getInstance() {
    if (instance == null) {
      instance = new GroundIntake();
    }
    return instance;
  }

  public void roll(){
    intakeMotor.set(IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void outRoll() {
    intakeMotor.set(-IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void stopRoll() {
    intakeMotor.set(0);
  }

  public void rawIntake(double s) {
    if (!auto) {
      intakeMotor.set(IntakeConstants.MAX_INTAKE_SPEED);
    }
  }

  /**
   * Stops the motor
   */
  public void stopIntake() {
    intakeMotor.set(0);
  }
  public void stopFlipper() {
    flipperMotor.set(0);
  }

  public void rawFlipUp(double s){
    flipperMotor.set(s * IntakeConstants.MAX_FLIP_SPEED);
  }

  public void rawFlipDown(double s) {
    flipperMotor.set(s * -IntakeConstants.MAX_FLIP_SPEED);
  }

  public RelativeEncoder getFlipperEncoder() {
    return flipperEncoder;
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
