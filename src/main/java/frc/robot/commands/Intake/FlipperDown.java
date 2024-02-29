// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntake;

public class FlipperDown extends Command {

  private static final GroundIntake m_intake = GroundIntake.getInstance();
  /** Creates a new Flipper. */
  public FlipperDown() {
    addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.rawFlipDown(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopFlipper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_intake.getFlipperEncoder().getPosition() <= Constants.IntakeConstants.FLIPPER_DOWN_POS)
      return true;
    return false;
  }
}
