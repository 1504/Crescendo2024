// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class InstantShooter extends Command {
  private final Shooter m_shooter = Shooter.getInstance();

  /** Creates a new Intake. */
  public InstantShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.shoot();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_shooter.stopShoot();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
