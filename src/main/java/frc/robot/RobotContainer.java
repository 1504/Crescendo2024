// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.Tank;
import frc.robot.commands.Turtles;
// import frc.robot.commands.Turtle2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShuffleboardManager;
import frc.robot.subsystems.ShuffleboardManager;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = Drivetrain.getInstance();
  public final ShuffleboardManager m_ShuffleboardManager = ShuffleboardManager.getInstance();
  private final Joystick _joystickOne = m_ShuffleboardManager.getJoystick(); //Controller for translation
  private final Limelight m_limelight= Limelight.getInstance();

  public final ShuffleboardManager m_shuffleboardManager = ShuffleboardManager.getInstance();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    autoChooser.addOption("BLUE1", loadPath());

    Shuffleboard.getTab("Auto").add(autoChooser);
    //SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command loadPath(String filename) {
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Nao pode abrir a trajetoria" + filename, exception.getStackTrace());
      System.out.println("Nao pode ler o arquivo" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, driveTrainSubsystem::getPose,
      new RamseteController(DriveTrainConstants.kRamseteB, DriveTrainConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeters,
          DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
      DriveTrainConstants.kDriveKinematics, driveTrainSubsystem::getWheelSpeeds,
      new PIDController(DriveTrainConstants.kpDriveVel, 0, 0),
      new PIDController(DriveTrainConstants.kpDriveVel, 0, 0), driveTrainSubsystem::tankDriveVolts,
      driveTrainSubsystem);

      return new SequentialCommandGroup(
        new InstantCommand(() -> driveTrainSubsystem.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_drive.setDefaultCommand(new Tank(() -> _joystickOne.getX(),() -> _joystickOne.getY()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}