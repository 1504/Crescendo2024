// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {
    public static final int LEFT1 = 2;
    public static final int LEFT2 = 4;
    public static final int RIGHT1 = 1;
    public static final int RIGHT2 = 3;
    public static final double DEADBAND = 0.03;

  }

  public final class IOConstants {
    public static final int JOYSTICK_ONE = 0;
  }

  public static class OperatorConstants {
    int a = 1;
    public static final int kDriverControllerPort = 0;
  }

  public static class ShootConstants {
    public static final int TOP_SHOOTER = 0;
    public static final int BOTTOM_SHOOTER = 1;

    public static final double TOP_P = 0;
    public static final double TOP_I = 0;
    public static final double TOP_D = 0;

    public static final double BOTTOM_P = 0;
    public static final double BOTTOM_I = 0;
    public static final double BOTTOM_D = 0;
  }
  public static final class BuildConstants {
    public static final double WHEEL_TO_CENTER_SIDE_INCHES = 0; //NEED TO MEASURE
    public static final double WHEEL_TO_CENTER_FRONT_INCHES = 0; //NEED TO MEAUSRE
    public static final double INCHES_TO_METERS = 0;
    public static final double GR = 0;
    public static final double WHEEL_CIRCUMFERENCE = 0 * Math.PI;
    public static final double INCHES_PER_REVOLUTION = WHEEL_CIRCUMFERENCE / GR;
  }

  public static class LimelightConstants {

    //angles in radians
    //heights in inches

    public static final double L_height = 0;
    public static final double AT_height = 53.88;
    public static final double L_angle = 0;

  }

  public static class IntakeConstants {
    public static final int intakePort = 0;
    public static final double kP = 0;
  }

  public static class AutoConstants {
    public static final String [] PATHS = {"BLUE1", "BLUE2", "BLUE3", "RED1", "RED2", "RED3"};

    public static final boolean USE_VISION = false;
    public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 1;
    public static final double AUTO_MAX_ACCEL_METERS_PER_SECOND_SQUARED = 3.0;
    
  }
}
