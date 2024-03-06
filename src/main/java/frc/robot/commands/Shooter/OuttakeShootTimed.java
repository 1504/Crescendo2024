// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShootConstants;
import frc.robot.commands.Auto.WaitTimed;
import frc.robot.commands.Intake.IntakeVariable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OuttakeShootTimed extends SequentialCommandGroup {
  /** Creates a new OuttakeShootTimed. */
  public OuttakeShootTimed() {
    addCommands(
      new InstantShooter(ShootConstants.right_speed, ShootConstants.left_speed),
      new WaitTimed(.75),
      new IntakeVariable(-.5)
    );
  }

  
}
