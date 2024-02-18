// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShootConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.FlipFront;

import frc.robot.commands.PIDdrive;
import frc.robot.commands.Shooter;
import frc.robot.commands.Tank;
import frc.robot.commands.Turtles;
import frc.robot.commands.moveBackwards;
import frc.robot.commands.resetEncoders;
import frc.robot.controlboard.ControlBoard;
// import frc.robot.commands.Turtle2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PIDShooter;
import frc.robot.subsystems.ShuffleboardManager;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final ControlBoard m_ControlBoard = ControlBoard.getInstance();
  private final Joystick _joystickOne = m_ControlBoard.getJoystick();

  private final XboxController _XboxController = m_ControlBoard.getXboxController();

  private final Limelight m_limelight= Limelight.getInstance();

  public final ShuffleboardManager m_shuffleboardManager = ShuffleboardManager.getInstance();

  public static final HashMap<String, Command> m_eventMap = new HashMap<>();
  private final SendableChooser<Command> m_autoChooser;

  private final PIDShooter m_shooter = PIDShooter.getInstance();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
  
    m_autoChooser = AutoBuilder.buildAutoChooser("b1_auto");
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
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
    m_drive.setDefaultCommand(new Tank(m_ControlBoard::getForward, m_ControlBoard::getRot));

    //new JoystickButton(_joystickOne, 2).onTrue(new moveBackwards(3));
    new JoystickButton(_XboxController, XboxController.Button.kB.value).whileTrue(new PIDdrive(2));
    new JoystickButton(_XboxController, XboxController.Button.kA.value).whileTrue(new resetEncoders());

    new JoystickButton(_XboxController, XboxController.Button.kY.value).whileTrue(new Shooter(m_shooter, ShootConstants.right_speed, ShootConstants.left_speed));
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
    System.err.println(" ----------------------------------------");
    System.err.println(m_autoChooser.getSelected());

    //m_autoChooser.getSelected().andThen(new AutoDrive(2, false));
    return m_autoChooser.getSelected();
    //return m_autoChooser.getSelected().andThen(new moveBackwards(2));
  }
}