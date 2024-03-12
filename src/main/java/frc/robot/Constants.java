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
    public static final int LEFT1 = 1;
    public static final int LEFT2 = 2;
    public static final int RIGHT1 = 3;
    public static final int RIGHT2 = 4;
    public static final double DEADBAND = 0.03;

    //voltage constraints
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 1;
    public static final double kaVoltSecondsSquaredPerMeter = 1;
  }


  public final class IOConstants {
    public static final int JOYSTICK_ONE = 0;
    public static final int JOYSTICK_TWO = 1;
    public static final int XBOX_CONTROLLER_GADGETS = 1;
    public static final int XBOX_CONTROLLER_DRIVE = 2;
  }

  public static class OperatorConstants {
    int a = 1;
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants {
    public static final int LEFT_ARM = 5;
    public static final int RIGHT_ARM = 6;


    public static final double ARM_UP_LEFT_POS = 60; //test
    public static final double ARM_UP_RIGHT_POS = 110;
    public static final double ARM_DOWN_POS = 0;

    public static final double ARM_UP_SPEED = 7.5;
    public static final double ARM_DOWN_SPEED = -7.5;
  }

  public static class ShootConstants {
    public static final int RIGHT_SHOOTER = 11;
    public static final int LEFT_SHOOTER = 12;

    public static final double RIGHT_P = 2.8;
    public static final double RIGHT_I = 0;
    public static final double RIGHT_D = 0;

    public static final double LEFT_P = 2.8;
    public static final double LEFT_I = 0;
    public static final double LEFT_D = 0;

    public static final double left_speed = -8.5;
    public static final double right_speed = -8.5;
    
  }

  public static class PIDConstants {
    public static final double left_PID_kp = 0.1;


    public static final double right_PID_kp = 0.1;

    public static final double theta_PID_kp = 0.1;

  }

  public static final class BuildConstants {

    public static final double INCHES_TO_METERS = 0.0254;
    public static final double WHEEL_TO_CENTER_SIDE_INCHES = 0.26 / INCHES_TO_METERS; //NEED TO MEASURE
    public static final double WHEEL_TO_CENTER_FRONT_INCHES = 0.3175 / INCHES_TO_METERS;
  
    public static final double GR = 6.28333333333333;
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public static final double INCHES_PER_REVOLUTION = WHEEL_CIRCUMFERENCE / GR;

    //public static final DifferentialDriveKinematics _KINEMATICS = new DifferentialDriveKinematics(0);

  }

  public static class LimelightConstants {

    //angles in radians
    //heights in inches

    public static final double L_height = 0;
    public static final double AT_height = 53.88;
    public static final double L_angle = 0;

    public static final String limelightURL = "http://10.15.4.69:5800/stream.mjpg";

  }

  public static class IntakeConstants {
    public static final int INTAKE_PORT = 21;
    public static final double kP = 0;

    public static final double MAX_INTAKE_SPEED = 0.6;

    public static final boolean UP = true;
    public static final boolean DOWN = false;
  }

  public static class FlipperConstants {
    public static final int FLIPPER_PORT = 22;
    public static final double MAX_FLIP_SPEED = 0.8;
    public static final double FLIPPER_DOWN_POS = -36;
    public static final double FLIPPER_UP_POS =-2; 
  }

  public static class AutoConstants {

    public static final boolean USE_VISION = false;
    public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 1;
    public static final double AUTO_MAX_ACCEL_METERS_PER_SECOND_SQUARED = 3.0;
    public static final double AUTO_MAX_ROTAT_RADIANS_PER_SECOND= 3.1415/4;

    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;

    public static final boolean foward = false;
    public static final boolean backward = true;

    public static final boolean clockwise = true;
    public static final boolean counterClockwise = false;
  }
}
