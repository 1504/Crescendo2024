// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlipperConstants;
import frc.robot.subsystems.Flipper;

public class RawFlip extends Command {
  private final Flipper m_flipper = Flipper.getInstance();
  private boolean flipUp;
  private DoubleSupplier speed;
  /** Creates a new RawFlip. */
  public RawFlip(DoubleSupplier s) {
    speed = s;
    if(speed.getAsDouble() > 0)
      flipUp = true;
    else
      flipUp = false;
    addRequirements(m_flipper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(flipUp && m_flipper.getEncoder().getPosition() <= FlipperConstants.FLIPPER_UP_POS){
        m_flipper.rawFlipUp(speed.getAsDouble());
      } else if (!flipUp && m_flipper.getEncoder().getPosition() >= FlipperConstants.FLIPPER_DOWN_POS) {
        m_flipper.rawFlipDown(speed.getAsDouble());
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
    return false;
  }
}
