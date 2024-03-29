// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Tank extends Command {
  
  protected final DoubleSupplier _rotSpeed;
  protected final DoubleSupplier _forwardSpeed;
  protected final Drivetrain m_drivetrain = Drivetrain.getInstance();

  /** Creates a new Tank. */
  public Tank(DoubleSupplier f, DoubleSupplier r) {

    _rotSpeed = r;
    _forwardSpeed = f;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveTank(_forwardSpeed.getAsDouble(), _rotSpeed.getAsDouble());
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
