// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootConstants;
import frc.robot.commands.Intake.Intake;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.PIDShooter;

public class ShootAndFeed extends Command {
  private final PIDShooter m_Shooter = PIDShooter.getInstance();
  private final GroundIntake m_Intake = GroundIntake.getInstance();
  private final Timer m_timer;
  /** Creates a new ShootAndFeed. */
  public ShootAndFeed() {
    m_timer = new Timer();
    addRequirements(m_Shooter);
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.setLeft(ShootConstants.left_speed);
    m_Shooter.setRight(ShootConstants.right_speed);
    m_Shooter.shoot();

    while(m_timer.get() > 1.5) {
      m_Intake.outRoll();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.reset();
    m_Shooter.setLeft(0);
    m_Shooter.setRight(0);
    m_Intake.stopRoll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > 3;
  }
}
