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
    public static final int LEFT = 11;
    public static final int RIGHT = 12;
    public static final double DEADBAND = 0.3;
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
}
