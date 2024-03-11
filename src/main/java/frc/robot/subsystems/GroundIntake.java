// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder; 

  private static GroundIntake instance = null;

  private GroundIntake() {
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_PORT, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
  }

  public static GroundIntake getInstance() {
    if (instance == null) {
      instance = new GroundIntake();
    }
    return instance;
  }

  public void roll() {
    intakeMotor.set(IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void outRoll() {
    intakeMotor.set(-IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void stopRoll() {
    intakeMotor.set(0);
  }

  public void rawIntake(double s) {
    intakeMotor.set(IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void variableIntake(double s) {
    intakeMotor.set(s);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
