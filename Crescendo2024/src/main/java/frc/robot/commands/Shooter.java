// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.PIDShooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shooter extends CommandBase {
  private final PIDShooter m_intake;
  private final double topSpeed; // top speed of the intake
  private final double bottomSpeed; // bottom speed of the intake
  /** Creates a new Intake. */
  public Shooter(PIDShooter shooter, double topSpeed, double bottomSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = shooter;
    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.setTop(topSpeed);
    m_intake.setBottom(bottomSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setTop(0);
    m_intake.setBottom(0);
    m_intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}