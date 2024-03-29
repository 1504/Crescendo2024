// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class RawRightDown extends Command {

  public final Arm m_arm = Arm.getInstance();

  public RawRightDown() {
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_arm.rawRight(ArmConstants.ARM_DOWN_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.rawRight(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
