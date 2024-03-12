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
  private final double turnDegrees;
  private double currDegrees;

  private final boolean direction; // true = clockwise -- false = counter

  /** Creates a new AutoTurn. */
  public AutoTurn(double degrees, boolean d) {
    direction = d;
    turnDegrees = degrees;
    currDegrees = m_gyro.getYaw();
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_gyro.reset();
  }

  @Override
  public void execute() {
    /* 
    if(direction) {
      if(m_gyro.getYaw() >= -turnDegrees) {
        m_drive.turn(direction);
      }
    }
    else{
      if(m_gyro.getYaw() <= turnDegrees) {
        m_drive.turn(direction);
      }
    }
    */

    
    if (direction) {
      if (m_gyro.getRotation2d().getDegrees() >= -turnDegrees) {
        m_drive.turn(direction);
      }
    } 
    else {
      if (m_gyro.getRotation2d().getDegrees() <= turnDegrees) {
        m_drive.turn(direction);
      }
    } 
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
  }

  @Override
  public boolean isFinished() {
    /* 
    if(direction)
      return m_gyro.getYaw() <= -turnDegrees;
    else
      return m_gyro.getYaw() >= turnDegrees;
      */
    
    System.err.println(m_gyro.getRotation2d().getDegrees());
    if (direction)
      return m_gyro.getRotation2d().getDegrees() <= -turnDegrees;
    else
      return m_gyro.getRotation2d().getDegrees() >= turnDegrees;
      
  }
}
