// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Intake.FlipperDown;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {
  private final Timer _timer = new Timer();
  private static GroundIntake m_intake = GroundIntake.getInstance();
  private static Shooter m_shooter = Shooter.getInstance();

  /** Creates a new Intake. */
  public AutoShooter() {
    addRequirements(m_shooter, m_intake);
  }

  @Override
  public void initialize() {
    _timer.reset();
    _timer.start();
  }

  @Override
  public void execute(){
    m_shooter.shoot();
    if (_timer.get() > 1) {
      m_intake.outRoll();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShoot();
    m_intake.stopRoll();
    new FlipperDown();
  }

  @Override
  public boolean isFinished() {
    return _timer.get() > 2.5;
  }
}

