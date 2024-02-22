// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.GroundIntake;

public class RawFlip extends Command {
  private final GroundIntake m_intake = GroundIntake.getInstance();
  private boolean flipUp = false;
  /** Creates a new RawFlip. */
  public RawFlip(boolean flip) {
    flipUp = flip;
    addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(flipUp && m_intake.getFlipEncoder().getPosition() <= Constants.IntakeConstants.FLIPPER_DOWN_POS){
        m_intake.rawFlipUp();
      } else if(!flipUp && m_intake.getFlipEncoder().getPosition() >= Constants.IntakeConstants.FLIPPER_UP_POS) {
        m_intake.rawFlipDown();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotor2();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
