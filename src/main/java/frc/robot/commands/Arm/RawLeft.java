// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class RawLeft extends Command {
  private final Arm m_arm = Arm.getInstance();
  double speed;

  public RawLeft(double s) {
    speed = s;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (m_arm.getLeftEncoder().getPosition() < ArmConstants.ARM_UP_LEFT_POS)
      m_arm.rawLeft(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.rawLeft(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
