// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;


public class Turtles extends Tank {
  double div;
  public void execute() {
    m_drivetrain.driveTank((_xSpeed.getAsDouble()/div), (_ySpeed.getAsDouble()/div));
  }
  public Turtles(DoubleSupplier xSpeed, DoubleSupplier ySpeed, double speedDivisor) {
    super(xSpeed, ySpeed);
    div = speedDivisor;
  }
}