// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;


public class ArmRetract extends Command {
  private final Arm _arm = Arm.getInstance();
  /** Creates a new ArmExtend. */
  public ArmRetract() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _arm.contract();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _arm.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(_arm.getLeftEncoder().getPosition() <= Constants.ArmConstants.ARM_EXTEND_POS)
      return true;
    return false;
  }
}