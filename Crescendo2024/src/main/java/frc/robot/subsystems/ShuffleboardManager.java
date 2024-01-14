// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  
  ShuffleboardTab telemetry;

  ShuffleboardTab limelight;

  private GenericEntry leftEncoder;
  private GenericEntry distanceFromAT;
  private GenericEntry xPos;
  private GenericEntry yPos;


  /** Creates a new ShuffleboardManager. */
  private ShuffleboardManager() {
    shuffleboardInit();

  }

  public void shuffleboardInit(){
    try {
      telemetry = Shuffleboard.getTab("Telemetry");
      leftEncoder = telemetry.add("Left Encoder", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();


      limelight = Shuffleboard.getTab( "Limelight");
      distanceFromAT = limelight.add("Distance from AT", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();

    }
    catch (Exception e) {
      System.out.println("ShuffleboardManager error: " + e);
    }
  }

  public void shuffleboardUpdate() {

  }

  @Override
  public void periodic() {
    shuffleboardUpdate();
  }
}
