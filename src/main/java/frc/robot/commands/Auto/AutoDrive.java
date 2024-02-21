// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;

public class AutoDrive extends Command {
  private Drivetrain _drive = Drivetrain.getInstance();
  private double _distance;
  private boolean _forward;
  /** Creates a new AutoDrive.
   * @d is distance in meters
   * @forward is true if robot goes forward, backwards false
   */
  public AutoDrive(double d, boolean forward) {
    _distance = d;
    _forward = forward;
    addRequirements(_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_forward)
    {
      _drive.driveTank(0.6*AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, 0);
    }
    else
      _drive.driveTank(-0.6*AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.driveTank(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(_drive.getDistanceTraveled())>=(_distance-0.075);
  }
}
