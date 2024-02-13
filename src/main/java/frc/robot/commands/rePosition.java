// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class rePosition extends Command {

  protected final Limelight _limelight = Limelight.getInstance();
  protected final Drivetrain m_drivetrain = Drivetrain.getInstance();
  protected final double KpDistance = -0.1f;
  protected double current_dist = _limelight.getDist();
  protected final double desired_dist = 1;
  //TODO: decide desired_dist
  
  /** Creates a new rePosition. */
  public rePosition() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleAdjust = _limelight.getAngleAdjust();
    double distAdjust = _limelight.getDistAdjust();

    m_drivetrain.driveTank(distAdjust,angleAdjust );
  }

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
