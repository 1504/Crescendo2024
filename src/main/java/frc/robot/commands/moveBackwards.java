// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class moveBackwards extends Command {

  protected final Drivetrain m_drivetrain = Drivetrain.getInstance();
  protected final double dist;

  /** Creates a new moveBackwards. */
  public moveBackwards(double d) {
    // Use addRequirements() here to declare subsystem dep
    dist = d;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    while( m_drivetrain.getDistanceTraveled() <dist) {
      m_drivetrain.driveTank(AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, AutoConstants.AUTO_MAX_ROTAT_RADIANS_PER_SECOND);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
