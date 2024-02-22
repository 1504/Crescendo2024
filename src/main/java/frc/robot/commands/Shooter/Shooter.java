// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import frc.robot.subsystems.PIDShooter;
import edu.wpi.first.wpilibj2.command.Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class Shooter extends Command {
  private final PIDShooter m_shooter;
  private final double rightSpeed; // right speed of the intake
  private final double leftSpeed; // bottom speed of the intake
  /** Creates a new Intake. */
  public Shooter(PIDShooter shooter, double rightSpeed, double leftSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setRight(rightSpeed);
    m_shooter.setLeft(leftSpeed);
    System.err.println("shooting");
  }

  @Override
  public void execute(){
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setRight(0);
    m_shooter.setLeft(0);
    m_shooter.stopShoot();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

