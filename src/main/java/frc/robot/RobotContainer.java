// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.commands.Intake.*;
import frc.robot.commands.resetEncoders;
import frc.robot.commands.Arm.MoveArms;
import frc.robot.commands.Arm.RawLeft;
import frc.robot.commands.Arm.RawLeftDown;
import frc.robot.commands.Arm.RawRight;
import frc.robot.commands.Arm.RawRightDown;
import frc.robot.commands.Auto.AutoDrive;
import frc.robot.commands.Auto.AutoShooter;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Auto.ShootAndFeed;
import frc.robot.commands.Auto.moveBackwards;
import frc.robot.commands.Drive.PIDdrive;
import frc.robot.commands.Drive.Tank;
import frc.robot.commands.Shooter.Shooter;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PIDShooter;
import frc.robot.subsystems.ShuffleboardManager;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


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
  
  private final XboxController _GadgetsController = m_ControlBoard.getGadgetsController();
  private final XboxController _DriveController = m_ControlBoard.getDriveController();


  //private final Limelight m_limelight= Limelight.getInstance();

  public final ShuffleboardManager m_shuffleboardManager = ShuffleboardManager.getInstance();

  public static final HashMap<String, Command> m_eventMap = new HashMap<>();

  private final PIDShooter m_shooter = PIDShooter.getInstance();

  private final GroundIntake m_intake = GroundIntake.getInstance();

  private final Arm m_arm = Arm.getInstance();

  private final SendableChooser<Command> m_autoChooser;

  public ShuffleboardTab auto;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    auto = Shuffleboard.getTab("auto");
    m_autoChooser = new SendableChooser<Command>();
    auto.add("Auto Chooser", m_autoChooser);
    m_autoChooser.addOption("red1", red1());
    m_autoChooser.addOption("red2", red2());
    m_autoChooser.addOption("red3", red3());
    m_autoChooser.addOption("blue1", blue1());
    m_autoChooser.addOption("blue2", blue2());
    m_autoChooser.addOption("blue3", blue3());

    m_arm.resetEncoders();
    // Configure the trigger bindings
    configureBindings();
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
    m_intake.setDefaultCommand(new RawFlip(m_ControlBoard::getIntakeUp)); //fliper up and down
    m_arm.setDefaultCommand(new MoveArms(m_ControlBoard::getArmsUp)); //arms up and down

    //new JoystickButton(_GadgetsController, XboxController.Button.kLeftStick.value).whileTrue(new RawFlip(IntakeConstants.DOWN)); //flip down
    //new JoystickButton(_GadgetsController, XboxController.Button.kRightStick.value).whileTrue(new RawFlip(IntakeConstants.UP)); //flip up
    new JoystickButton(_GadgetsController, XboxController.Button.kLeftBumper.value).onTrue(new FlipperDown());
    new JoystickButton(_GadgetsController, XboxController.Button.kRightBumper.value).onTrue(new FlipperUp());
    new JoystickButton(_GadgetsController, XboxController.Button.kY.value).whileTrue(new Shooter(ShootConstants.right_speed, ShootConstants.left_speed));
    //new JoystickButton(_GadgetsController, XboxController.Button.kA.value).onTrue(new ShootAndFeed());
    new JoystickButton(_GadgetsController, XboxController.Button.kB.value).whileTrue(new Intake());


    new JoystickButton(_DriveController, XboxController.Button.kBack.value).whileTrue(new RawLeft(0.5));
    new JoystickButton(_DriveController, XboxController.Button.kStart.value).whileTrue(new RawRight(0.5));
    new JoystickButton(_DriveController, XboxController.Button.kLeftBumper.value).whileTrue(new RawLeftDown());
    new JoystickButton(_DriveController, XboxController.Button.kRightBumper.value).whileTrue(new RawRightDown());
    //new JoystickButton(_DriveController, XboxController.Button.kY.value).onTrue(new PIDdrive(1));
    new JoystickButton(_DriveController, XboxController.Button.kB.value).whileTrue(new Outtake());
    new JoystickButton(_DriveController, XboxController.Button.kA.value).whileTrue(new resetEncoders());
  }

  private void initAuton() {
  }
// test these
  public Command red1() {
    return (new AutoShooter(m_shooter, m_intake, 5)
      .andThen(new AutoDrive(1, false))
      .andThen(new AutoTurn(false)))
      .andThen(new AutoDrive(1, false));
  }

  public Command red2() {
    return (new AutoShooter(m_shooter, m_intake, 5)
      .andThen(new AutoDrive(1, false)));
  }

  public Command red3() {
    return (new AutoShooter(m_shooter, m_intake, 5)
      .andThen(new AutoDrive(1, false))
      .andThen(new AutoTurn(true)))
      .andThen(new AutoDrive(1, false));
  }

  public Command blue1() {
    return (new AutoShooter(m_shooter, m_intake, 5)
      .andThen( new AutoDrive(1, false))
      .andThen( new AutoTurn(true))
      .andThen(new AutoDrive(1, false)));
  }

  public Command blue2() {
    return ( new AutoShooter(m_shooter, m_intake, 5)
      .andThen(new AutoDrive(1, false)));
  }

  public Command blue3() {
    return (new AutoShooter(m_shooter, m_intake, 5)
      .andThen(new AutoDrive(1, false))
      .andThen(new AutoTurn(false))
      .andThen(new AutoDrive(1, false)));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
    /*
    Timer timer = new Timer();
    timer.start();
    BooleanSupplier sup = () -> timer.get() >= 5;
    
    System.err.println(" ----------------------------------------");
    //auto turn -- true = clockwise; false = counter
    
    return (new AutoShooter(m_shooter, m_intake, 5)
      .andThen(new AutoDrive(1, false))
      .andThen(new AutoTurn(false)))
      .andThen(new AutoDrive(1, false));
    //return new AutoShooter(m_shooter, m_intake, 5).andThen(new AutoDrive(1.5, false));
    */
  }
}