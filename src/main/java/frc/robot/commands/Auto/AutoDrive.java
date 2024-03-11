// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants.AutoConstants;

public class AutoDrive extends Command {
  private Drivetrain _drive = Drivetrain.getInstance();
  private double _distance;
  private boolean _forward;

  public AutoDrive(double d, boolean forward) {
    _distance = d;
    _forward = forward;
    addRequirements(_drive);
  }

  @Override
  public void initialize() {
    _drive.resetEncoders();
  }

  @Override
  public void execute() {
    if (_forward) {
      _drive.driveTank(-0.6 * AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, 0);
    } else
      _drive.driveTank(0.6 * AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, 0);
  }

  @Override
  public void end(boolean interrupted) {
    _drive.driveTank(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(_drive.getDistanceTraveled()) >= (_distance);
  }
}
