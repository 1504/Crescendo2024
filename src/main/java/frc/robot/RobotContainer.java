// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.resetEncoders;
import frc.robot.commands.Arm.MoveArms;
import frc.robot.commands.Arm.RawLeft;
import frc.robot.commands.Arm.RawLeftDown;
import frc.robot.commands.Arm.RawRight;
import frc.robot.commands.Arm.RawRightDown;
import frc.robot.commands.Auto.AutoDrive;
import frc.robot.commands.Auto.AutoIntake;
import frc.robot.commands.Auto.AutoShooter;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Drive.Tank;
import frc.robot.commands.Intake.FlipperDown;
import frc.robot.commands.Intake.FlipperUp;
import frc.robot.commands.Intake.InstantIntake;
import frc.robot.commands.Intake.Intake;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.Intake.RawFlip;
import frc.robot.commands.Shooter.OuttakeShootTimed;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShuffleboardManager;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = Drivetrain.getInstance();
  public final ShuffleboardManager m_ShuffleboardManager = ShuffleboardManager.getInstance();
  private final ControlBoard m_ControlBoard = ControlBoard.getInstance();

  private final XboxController _GadgetsController = m_ControlBoard.getGadgetsController();
  private final XboxController _DriveController = m_ControlBoard.getDriveController();

  // private final Limelight m_limelight= Limelight.getInstance();

  public final ShuffleboardManager m_shuffleboardManager = ShuffleboardManager.getInstance();

  public static final HashMap<String, Command> m_eventMap = new HashMap<>();

  private final Shooter m_shooter = Shooter.getInstance();

  private final GroundIntake m_intake = GroundIntake.getInstance();

  private final Flipper m_flipper = Flipper.getInstance();

  private final Arm m_arm = Arm.getInstance();

  private final SendableChooser<Command> m_autoChooser;

  public ShuffleboardTab auto;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    auto = Shuffleboard.getTab("auto");
    m_autoChooser = new SendableChooser<Command>();
    auto.add("Auto Chooser", m_autoChooser).withSize(2,1);
    m_autoChooser.addOption("red amp 2", redAmp(2));
    m_autoChooser.addOption("red middle 2", redMiddle(2));
    m_autoChooser.addOption("red source 2", redSource(2));
    m_autoChooser.addOption("blue amp 2", blueAmp(2));
    m_autoChooser.addOption("blue middle 2", blueMiddle(2));
    m_autoChooser.addOption("blue source 2", blueSource(2));

    m_autoChooser.addOption("red amp 1", redAmp(1));
    m_autoChooser.addOption("red middle 1", redMiddle(1));
    m_autoChooser.addOption("red source 1", redSource(1));
    m_autoChooser.addOption("blue amp 1", blueAmp(1));
    m_autoChooser.addOption("blue middle 1", blueMiddle(1));
    m_autoChooser.addOption("blue source 1", blueSource(1));

    m_arm.resetEncoders();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drive.setDefaultCommand(new Tank(m_ControlBoard::getForward, m_ControlBoard::getRot));//drive
    m_flipper.setDefaultCommand(new RawFlip(m_ControlBoard::getIntakeUp)); // fliper up and down
    m_arm.setDefaultCommand(new MoveArms(m_ControlBoard::getArmsUp)); // arms up and down

    // ARMS UP
    new JoystickButton(_DriveController, XboxController.Button.kBack.value)
        .whileTrue(new RawLeft(ArmConstants.ARM_UP_SPEED));
    new JoystickButton(_DriveController, XboxController.Button.kStart.value)
        .whileTrue(new RawRight(ArmConstants.ARM_UP_SPEED));

    // ARMS DOWN
    new JoystickButton(_DriveController, XboxController.Button.kLeftBumper.value)
      .whileTrue(new RawLeftDown());
    new JoystickButton(_DriveController, XboxController.Button.kRightBumper.value)
      .whileTrue(new RawRightDown());

    new JoystickButton(_DriveController, XboxController.Button.kA.value).whileTrue(new resetEncoders());

    // INTAKE / OUTTAKE
    new JoystickButton(_DriveController, XboxController.Button.kB.value).whileTrue(new Outtake());

    new JoystickButton(_GadgetsController, XboxController.Button.kB.value).whileTrue(new Intake());

    // SHOOTER
    JoystickButton y = new JoystickButton(_GadgetsController, XboxController.Button.kY.value);
    y.onTrue(new OuttakeShootTimed()); // While held
    y.onFalse(new ParallelCommandGroup(new StopShooter(), new InstantIntake(0))); // When released

    // FLIPPER
    new JoystickButton(_GadgetsController, XboxController.Button.kLeftBumper.value).onTrue(new FlipperDown());
    new JoystickButton(_GadgetsController, XboxController.Button.kRightBumper.value).onTrue(new FlipperUp());

    // MANUEL SHOOTER
    new JoystickButton(_GadgetsController, XboxController.Button.kX.value).whileTrue(new Shoot());
    new JoystickButton(_GadgetsController, XboxController.Button.kBack.value).whileTrue(new Outtake());
  }

  public Command redAmp(int notes) {
    if (notes == 2) {
      return (new AutoShooter()
          .andThen(new AutoDrive(0.35, false))
          .andThen(new FlipperDown())
          .andThen(new AutoTurn(38, false))
          .andThen(Commands.parallel(new AutoDrive(2, false), new AutoIntake(1)))
          .andThen(new WaitCommand(0.2))
          .andThen(Commands.parallel(new AutoDrive(2, true), new FlipperUp()))
          .andThen(new AutoTurn(38, true))
          .andThen(new AutoDrive(0.35, true))
          .andThen(new AutoShooter()));
    } else if (notes == 1) {
      return new AutoShooter()
          .andThen(new AutoDrive(0.5, false))
          .andThen(new AutoTurn(38, false))
          .andThen(new AutoDrive(2, false));
    }
    return null;
  }

  public Command redMiddle(int notes) {
    if (notes == 2) {
      return (new AutoShooter()
          .andThen(new FlipperDown())
          .andThen(Commands.parallel(new AutoIntake(0.5), new AutoDrive(1, false)))
          .andThen(Commands.parallel(new AutoDrive(1, true), new FlipperUp())
              .andThen(new AutoShooter())));
    } else if (notes == 1) {
      return new AutoShooter()
          .andThen(new AutoDrive(2, false));
    }
    return null;
  }

  public Command redSource(int notes) {
    if (notes == 2) {
      return (new AutoShooter()
          .andThen(new AutoDrive(0.35, false))
          .andThen(new AutoTurn(38, true)))
          .andThen(Commands.parallel(new AutoIntake(0), new AutoDrive(1.3, false), new FlipperDown()))
          .andThen(new WaitCommand(0.1))
          .andThen(Commands.parallel(new AutoDrive(1.3, true), new FlipperUp()))
          .andThen(new AutoTurn(38, false))
          .andThen(new WaitCommand(0.1))
          .andThen(new AutoDrive(0.35, true))
          .andThen(new AutoShooter());
    } else if (notes == 1) {
      return new AutoShooter()
          .andThen(new AutoDrive(0.5, false))
          .andThen(new AutoTurn(38, true))
          .andThen(new AutoDrive(2, false));
    }
    return null;
  }

  public Command blueSource(int notes) {
    if (notes == 2) {
      return (new AutoShooter()
          .andThen(new AutoDrive(0.35, false))
          .andThen(new FlipperDown())
          .andThen(new AutoTurn(38, false))
          .andThen(Commands.parallel(new AutoDrive(1.3, false), new AutoIntake(1)))
          .andThen(new WaitCommand(0.2))
          .andThen(Commands.parallel(new AutoDrive(1.3, true), new FlipperUp()))
          .andThen(new AutoTurn(38, true))
          .andThen(new AutoDrive(0.35, true))
          .andThen(new AutoShooter()));
    } else if (notes == 1) {
      return new AutoShooter()
          .andThen(new AutoDrive(0.5, false))
          .andThen(new AutoTurn(38, false))
          .andThen(new AutoDrive(2, false));
    }
    return null;
  }

  public Command blueMiddle(int notes) {
    if (notes == 2) {
      return (new AutoShooter()
          .andThen(new FlipperDown())
          .andThen(Commands.parallel(new AutoIntake(0.5), new AutoDrive(1, false)))
          .andThen(Commands.parallel(new AutoDrive(1, true), new FlipperUp())
              .andThen(new AutoShooter())));
    } else if (notes == 1) {
      return new AutoShooter()
          .andThen(new AutoDrive(2, false));
    }
    return null;
  }

  public Command blueAmp(int notes) {
    if (notes == 2) {
      return (new AutoShooter()
          .andThen(new AutoDrive(0.35, false))
          .andThen(new AutoTurn(38, true)))
          .andThen(Commands.parallel(new AutoIntake(0), new AutoDrive(2, false), new FlipperDown()))
          .andThen(new WaitCommand(0.1))
          .andThen(Commands.parallel(new AutoDrive(2, true), new FlipperUp()))
          .andThen(new AutoTurn(38, false))
          .andThen(new WaitCommand(0.1))
          .andThen(new AutoDrive(0.35, true))
          .andThen(new AutoShooter());
    } else if (notes == 1) {
      return new AutoShooter()
          .andThen(new AutoDrive(0.5, false))
          .andThen(new AutoTurn(38, true))
          .andThen(new AutoDrive(2, false));
    }
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}