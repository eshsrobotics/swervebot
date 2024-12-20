// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;

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
  }
}
