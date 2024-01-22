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

public class ShuffleboardManager extends SubsystemBase {

  private static ShuffleboardManager _instance = null;

  public static ShuffleboardManager getInstance(){
    if( _instance == null ) {
      _instance = new ShuffleboardManager();
    }
    return _instance;
  }

  private final Drivetrain _drive = Drivetrain.getInstance();
  private final Limelight _limelight = Limelight.getInstance();
  private final Joystick _joystickOne = new Joystick(IOConstants.JOYSTICK_ONE);
  
  ShuffleboardTab telemetry;
  ShuffleboardTab limelight;

  //telemetry
  private GenericEntry leftEncoder;
  private GenericEntry rightEncoder;
  private GenericEntry xPos;
  private GenericEntry yPos;
  private GenericEntry joystickX;
  private GenericEntry joystickY;

  //limelight
  private GenericEntry distanceFromAT;


  /** Creates a new ShuffleboardManager. */
  private ShuffleboardManager() {
    shuffleboardInit();
  }

  public void shuffleboardInit(){
    try {
      telemetry = Shuffleboard.getTab("Telemetry");
      leftEncoder = telemetry.add("Left Encoder", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
      rightEncoder = telemetry.add("Right Encoder", 2).withPosition(2, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
      
      limelight = Shuffleboard.getTab( "Limelight");
      distanceFromAT = limelight.add("Distance from AT", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();

      joystickX = telemetry.add("Joystick X",0).withPosition(0,2).withSize(2,1).withWidget(BuiltInWidgets.kTextView).getEntry();
      joystickY = telemetry.add("Joystick Y",0).withPosition(2,2).withSize(2,1).withWidget(BuiltInWidgets.kTextView).getEntry();  
    }
    catch (Exception e) {
      System.out.println("ShuffleboardManager error: " + e);
    }
  }

  public Joystick getJoystick() {
    return _joystickOne;
  }

  public void shuffleboardUpdate() {
    leftEncoder.setDouble( _drive.getLeftVelocity());
    rightEncoder.setDouble(_drive.getRightVelocity());

    distanceFromAT.setDouble(_limelight.getDist());

    joystickX.setDouble(_joystickOne.getX());
    joystickY.setDouble(_joystickOne.getY());

  }

  @Override
  public void periodic() {
    shuffleboardUpdate();
  }
}