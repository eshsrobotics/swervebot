// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    /**
     * Dead zone for the joystick -- or, better to say, for the
     * {@link SlewRateLimiter} that we associate with each joystick.
     *
     * Any joystick inputs with absolute values less than this will be treated
     * as if they are 0.
     */
    public static final double JOYSTICK_DEAD_ZONE = 0.02;
  }

  public static class DriveConstants {
    /**
     * The radius of your swerve wheels.  Used in calculations to determine
     * velocity of each module in meters per second.
     *
     * <p>TODO: Measure the real value for this number,</p>
     */
    public static final double SWERVE_MODULE_WHEEL_RADIUS_METERS = 0.0508;

    /**
     * Theoretical maximum of chassis speed in meters per second for kinematic
     * conversion purposes.
     */
    public static final double SWERVE_DRIVE_MAX_DRIVING_SPEED = 0.5;

    /**
     * Theoretical maximum turning speed of the robot in radians per second for
     * kinematic conversion purposes.
     */
    public static final double SWERVE_DRIVE_MAX_TURNING_SPEED = 0.4;

    /**
     * The positions of the wheels on the Chassis measured in meters. (needs testing)
     */
    public static final Translation2d SWERVE_MODULE_POSITION_FRONT_RIGHT = new Translation2d(0.3, 0.3);
    public static final Translation2d SWERVE_MODULE_POSITION_FRONT_LEFT = new Translation2d(0.3, -0.3);
    public static final Translation2d SWERVE_MODULE_POSITION_BACK_RIGHT = new Translation2d(-0.3, 0.3);
    public static final Translation2d SWERVE_MODULE_POSITION_BACK_LEFT = new Translation2d(-0.3, -0.3);

    /**
     * The type of drive that the {@link DriveSubsystem} will use.
     */
    public enum DriveType {
      SWERVE_DRIVE,
      DIFFERENTIAL_DRIVE
    }

    public static final int DIFFERENTIAL_DRIVE_PWM_LEFT_MOTOR_1 = 1;
    public static final int DIFFERENTIAL_DRIVE_PWM_LEFT_MOTOR_2 = 2;
    public static final int DIFFERENTIAL_DRIVE_PWM_RIGHT_MOTOR_1 = 3;
    public static final int DIFFERENTIAL_DRIVE_PWM_RIGHT_MOTOR_2 = 4;
  }
}
