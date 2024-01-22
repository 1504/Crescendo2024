// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;

public class Intake extends Command {
  private final GroundIntake m_intake;
  /** Creates a new Intake. */
  public Intake(GroundIntake _intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = _intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.roll(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
