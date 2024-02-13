// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {

  private static Limelight _instance = null;
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static Gyroscope _gyro = Gyroscope.getInstance();

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
    }

  public static void setConfig(String varName, int value) {
    table.getEntry(varName).setNumber(value);
  }

  public static double getConfig(String varName) {
    return table.getEntry(varName).getDouble(0.0);
  }

  public static boolean hasTarget() {
		return table.getEntry("tv").getDouble(0) == 1;
	}

  public Pose2d getFieldPose() {
    return new Pose2d(_gyro.getDisplacementX(), _gyro.getDisplacementY(), _gyro.getYawRotation());
  }

  public double getAngleAdjust() {
    double steering_adjust = 0.0f;
    double tx = getConfig("tx");
    double heading_error = -tx;
    if( tx >1.0) {
      steering_adjust = LimelightConstants.KpAim *heading_error - LimelightConstants.min_aim_command;
    }
    else if (tx <-1.0) {
      steering_adjust = LimelightConstants.KpAim*heading_error + LimelightConstants.min_aim_command;
    }
    return steering_adjust;
  }

  public double getDistAdjust() {
    double ty = getConfig("ty");
    double dist_error = -ty;
    double dist_adjust = LimelightConstants.KpDistance * dist_error;

    return dist_adjust;
  }


  public double getDist() {
    double h_diff = LimelightConstants.AT_height-LimelightConstants.L_height;
    double tan = Math.tan(LimelightConstants.L_angle+table.getEntry("ty").getDouble(0.0));

    return h_diff/tan;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
