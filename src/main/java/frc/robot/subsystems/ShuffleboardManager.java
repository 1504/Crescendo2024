// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import frc.robot.controlboard.ControlBoard;

public class ShuffleboardManager extends SubsystemBase {

  private static ShuffleboardManager _instance = null;

  public static ShuffleboardManager getInstance() {
    if (_instance == null) {
      _instance = new ShuffleboardManager();
    }
    return _instance;
  }

  private final Drivetrain _drive = Drivetrain.getInstance();
  private final Limelight _limelight = Limelight.getInstance();
  private final Gyroscope _gyroOne = Gyroscope.getInstance();
  private final ControlBoard _controlboard = ControlBoard.getInstance();

  ShuffleboardTab telemetry;
  ShuffleboardTab limelight;
  ShuffleboardTab _Gyroscope;

  // telemetry
  private GenericEntry leftEncoder;
  private GenericEntry rightEncoder;
  private GenericEntry xPos;
  private GenericEntry yPos;
  private GenericEntry joystickX;
  private GenericEntry joystickY;
  private GenericEntry GyroPitch;
  private GenericEntry GyroRoll;
  private GenericEntry GyroYaw;
  private GenericEntry GyroDX;
  private GenericEntry GyroDY;
  private GenericEntry GyroDZ;

  // limelight
  private GenericEntry distanceFromAT;

  /** Creates a new ShuffleboardManager. */
  private ShuffleboardManager() {
    shuffleboardInit();
  }

  public void shuffleboardInit() {
    try {
      telemetry = Shuffleboard.getTab("Telemetry");
      _Gyroscope = Shuffleboard.getTab("Gyroscope");
      leftEncoder = telemetry.add("Left Encoder", 0).withPosition(0, 0).withSize(2, 2)
          .withWidget(BuiltInWidgets.kGraph).getEntry();
      rightEncoder = telemetry.add("Right Encoder", 2).withPosition(2, 0).withSize(2, 2)
          .withWidget(BuiltInWidgets.kGraph).getEntry();

      limelight = Shuffleboard.getTab("Limelight");
      distanceFromAT = limelight.add("Distance from AT", 0).withPosition(0, 0).withSize(2, 2)
          .withWidget(BuiltInWidgets.kTextView).getEntry();

      joystickX = telemetry.add("Joystick X", 0).withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      joystickY = telemetry.add("Joystick Y", 0).withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      GyroPitch = _Gyroscope.add("Gyro Pitch", 0).withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      GyroRoll = _Gyroscope.add("Gyro Roll", 0).withPosition(2, 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      GyroYaw = _Gyroscope.add("Gyro Yaw", 0).withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      GyroDX = _Gyroscope.add("Gyro Displacement X", 0).withPosition(2, 1).withSize(2, 1)
          .withWidget(BuiltInWidgets.kTextView).getEntry();
      GyroDY = _Gyroscope.add("Gyro Displacement Y", 0).withPosition(0, 2).withSize(2, 1)
          .withWidget(BuiltInWidgets.kTextView).getEntry();
      GyroDZ = _Gyroscope.add("Gyro Displacement Z", 0).withPosition(2, 2).withSize(2, 1)
          .withWidget(BuiltInWidgets.kTextView).getEntry();
    } catch (Exception e) {
      System.out.println("ShuffleboardManager error: " + e);
    }
  }

  public void shuffleboardUpdate() {
    leftEncoder.setDouble(_drive.getLeftVelocity());
    rightEncoder.setDouble(_drive.getRightVelocity());

    distanceFromAT.setDouble(_limelight.getDist());

    joystickX.setDouble(_controlboard.getX());
    joystickY.setDouble(_controlboard.getY());
    GyroPitch.setDouble(_gyroOne.getPitch());
    GyroRoll.setDouble(_gyroOne.getRoll());
    GyroYaw.setDouble(_gyroOne.getYaw());
    GyroDX.setDouble(_gyroOne.getDisplacementX());
    GyroDY.setDouble(_gyroOne.getDisplacementY());
    GyroDZ.setDouble(_gyroOne.getDisplacementZ());
  }

  @Override
  public void periodic() {
    shuffleboardUpdate();
  }
}