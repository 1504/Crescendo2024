// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlipperConstants;
import frc.robot.subsystems.Flipper;

public class FlipperUp extends Command {

  private static final Flipper m_flipper = Flipper.getInstance();
  /** Creates a new FlipperUp. */
  public FlipperUp() {
    addRequirements(m_flipper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flipper.rawFlipUp(0.6);
    if (m_flipper.getEncoder().getPosition() <= FlipperConstants.FLIPPER_UP_POS + 12)
    {
      m_flipper.rawFlipUp(0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flipper.stopFlipper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_flipper.getEncoder().getPosition() <= FlipperConstants.FLIPPER_UP_POS)
      return true;
    return false;
  }
}
