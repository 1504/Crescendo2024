// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;
import frc.robot.subsystems.PIDShooter;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.commands.Intake.FlipperDown;
import frc.robot.subsystems.GroundIntake;
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj2.command.Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see: 
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoShooter extends Command {
  private final PIDShooter m_shooter;// bottom speed of the intake
  private final GroundIntake m_intake;
  private final Timer _timer;
  private final double _time;
  /** Creates a new Intake. */
  public AutoShooter(PIDShooter shooter, GroundIntake intake, double time) {
    _timer = new Timer();
    _time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    _timer.reset();
    _timer.start();
    m_shooter.setRight(ShootConstants.right_speed);
    m_shooter.setLeft(ShootConstants.left_speed);
    m_shooter.shoot();
  }

  @Override
  public void execute(){
    if (_timer.get() > 1 && _timer.get() <10) {
      m_intake.outRoll();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setRight(0);
    m_shooter.setLeft(0);
    m_shooter.stopShoot();
    m_intake.stopRoll();
    new FlipperDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _timer.get() > _time;
  }
}

