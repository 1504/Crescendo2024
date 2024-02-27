// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyroscope;

public class AutoTurn extends Command {

  private final Drivetrain m_drive = Drivetrain.getInstance();
  private final Gyroscope m_gyro = Gyroscope.getInstance();

  private final boolean direction; //true = clockwise -- false = counter

  /** Creates a new AutoTurn. */
  public AutoTurn(boolean d) {
    // Use addRequirements() here to declare subsystem dependencies.
    direction = d;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( direction) {
      System.err.println(m_gyro.getRotation2d().getDegrees());
      if( m_gyro.getRotation2d().getDegrees() > -57 ) {
        m_drive.turn(direction);
      }
    }
    else {
      if( m_gyro.getRotation2d().getDegrees() < 57 ) {
        m_drive.turn(direction);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(direction)
      return m_gyro.getRotation2d().getDegrees() <= -57;
    else
      return m_gyro.getRotation2d().getDegrees() >= 57;
  }
}
