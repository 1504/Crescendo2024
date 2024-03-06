// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import frc.robot.subsystems.PIDShooter;
import edu.wpi.first.wpilibj2.command.Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class StopShooter extends Command {
  private final PIDShooter m_shooter = PIDShooter.getInstance();
  /** Creates a new Intake. */
  public StopShooter() {
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.stopShoot();
  }

  @Override
  public void execute(){
  }

  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

