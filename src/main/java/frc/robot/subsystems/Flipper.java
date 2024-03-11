// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlipperConstants;

public class Flipper extends SubsystemBase {
  private final CANSparkMax flipperMotor;// = new CANSparkMax(IntakeConstants.intakePort_2, MotorType.kBrushless);
  private final RelativeEncoder flipperEncoder;// = flipperMotor.getEncoder();
  private static Flipper instance = null;

  public static Flipper getInstance() {
    if (instance == null) {
      instance = new Flipper();
    }
    return instance;
  }

  public Flipper() {
    flipperMotor = new CANSparkMax(FlipperConstants.FLIPPER_PORT, MotorType.kBrushless);
    flipperEncoder = flipperMotor.getEncoder();

    flipperMotor.setIdleMode(IdleMode.kBrake);
  }

  public void stopFlipper() {
    flipperMotor.set(0);
  }

  public void rawFlipUp(double s) {
    flipperMotor.set(s * FlipperConstants.MAX_FLIP_SPEED);
  }

  public void rawFlipDown(double s) {
    flipperMotor.set(s * -FlipperConstants.MAX_FLIP_SPEED);
  }

  public RelativeEncoder getEncoder() {
    return flipperEncoder;
  }

  @Override
  public void periodic() {
  }
}
