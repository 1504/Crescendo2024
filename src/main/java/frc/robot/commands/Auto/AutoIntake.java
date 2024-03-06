// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;

public class AutoIntake extends Command {
  private static GroundIntake m_intake = GroundIntake.getInstance();
  /** Creates a new AutoIntake. */
  private static Timer _timer = new Timer();
  private static double time;
  public AutoIntake(double t) {
    time = t;
    addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.reset();
    _timer.start();

    m_intake.roll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_timer.get() <time) {
      m_intake.roll();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopRoll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _timer.get() > 5;
  }
}
