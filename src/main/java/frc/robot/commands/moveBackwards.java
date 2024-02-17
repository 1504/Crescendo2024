// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
// pid
import edu.wpi.first.math.controller.PIDController;

public class moveBackwards extends Command {

  protected final Drivetrain m_drivetrain = Drivetrain.getInstance();
  protected final double dist;
  protected double error;

  // pid constants
  double kP = 0.2;
  double kI = 0.0;
  double kD = 0.0;

  // create PID controller
  private final PIDController m_pidController = new PIDController(kP, kI, kD);




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
    m_pidController.setSetpoint(error);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // error = error of distance
    // curr - target
    // set wheel speed to pid
    error = dist - m_drivetrain.getDistanceTraveled();

    //System.err.println(m_pidController.calculate(m_drivetrain.getDistanceTraveled()));
    
    //m_drivetrain.driveTank(m_pidController.calculate(m_drivetrain.getDistanceTraveled()), 0);
    
    
    if( -m_drivetrain.getDistanceTraveled() <dist) {
      m_drivetrain.driveTank(-AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND*0.4, 0);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getDistanceTraveled())>dist;
  }
}
