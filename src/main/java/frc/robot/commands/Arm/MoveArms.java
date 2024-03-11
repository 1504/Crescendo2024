// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArms extends Command {
  private DoubleSupplier speed;
  private final Arm m_arm = Arm.getInstance();

  public MoveArms(DoubleSupplier s) {
    speed = s;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_arm.extendBoth(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
