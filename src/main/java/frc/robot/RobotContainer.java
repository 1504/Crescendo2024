// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Tank;
import frc.robot.commands.Turtles;
import frc.robot.controlboard.ControlBoard;
// import frc.robot.commands.Turtle2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShuffleboardManager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.ResourceBundle.Control;
import java.util.function.BiConsumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPRamseteController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final ControlBoard m_ControlBoard = ControlBoard.getInstance();
  private final Joystick _joystickOne = m_ControlBoard.getJoystick();
  private final Limelight m_limelight= Limelight.getInstance();

  public final ShuffleboardManager m_shuffleboardManager = ShuffleboardManager.getInstance();

  public static final HashMap<String, Command> m_eventMap = new HashMap<>();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final AutoBuilder autoBuilder;

  private final List<List<PathPlannerPath>> m_testPaths = new ArrayList<>();
  {
    for (String path : AutoConstants.PATHS) {
      List<PathPlannerPath> pathes = PathPlannerAuto.getPathGroupFromAutoFile(path);
      ChassisSpeeds max_speeds = new ChassisSpeeds(AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, AutoConstants.AUTO_MAX_ROTAT_RADIANS_PER_SECOND);

      Rotation2d starting_rotation = new Rotation2d(0);
      m_testPaths.add(pathes);
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    Shuffleboard.getTab("Auto").add(m_autoChooser);
    
    autoBuilder = new AutoBuilder();

    for (int i = 0; i < m_testPaths.size(); i++) {
      if (i == 0) {
        m_autoChooser.setDefaultOption(AutoConstants.PATHS[i], getCommandFromPath(m_testPaths.get(i)));
      } else {
        //m_autoChooser.addOption(AutoConstants.PATHS[i], autoBuilder.fullAuto(m_testPaths.get(i)));
      }
    }

    Shuffleboard.getTab("Pregame").add("Auton Path", m_autoChooser)
        .withPosition(0, 1)
        .withSize(3, 1);
  }

   public SequentialCommandGroup getCommandFromPath(List<PathPlannerPath> paths) {    
    FollowPathRamsete r = new FollowPathRamsete(
              paths.get(0),
              m_drive::getPose,
              m_drive::getSpeeds,
              m_drive::consumerSpeeds,
              new ReplanningConfig(),
              m_drive::flipPath,
              m_drive
    );

    SequentialCommandGroup commands = new SequentialCommandGroup(r);
      
    for(int i = 1; i < paths.size(); i++) {
      r.andThen(
          new FollowPathRamsete(
              paths.get(i),
              m_drive::getPose,
              m_drive::getSpeeds,
              m_drive::consumerSpeeds,
              new ReplanningConfig(),
              m_drive::flipPath,
              m_drive
          )
      );
    }

    return commands;
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
    m_drive.setDefaultCommand(new Tank(() -> m_ControlBoard.getX(),() -> m_ControlBoard.getY()));
  }

  private void initAuton() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}