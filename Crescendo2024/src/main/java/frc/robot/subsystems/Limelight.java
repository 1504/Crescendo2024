// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private static Limelight _instance = null;
  private NetworkTable table;

  /** 
   * Ensures only one instance of Limelight is created
   * 
   * @return Limelight instance
   */

  public static Limelight getInstance() {
    if (_instance == null) {
      _instance = new Limelight();
    }
    return _instance;
  }

  /** Creates a new Limelight. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getVal(String varName) {
    return table.getEntry(varName).getDouble(0.0);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
