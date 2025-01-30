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

    /**
     * <p> We will have an array of four pivot motors and four drive motors. This
     * enumeration is indices for the arrays so that we know which drive motor
     * is in what location. </p>
     *
     * <p> If you see a shuffleboard display coming from PivotMotor2, then you will
     * know you are dealing with a BACK_RIGHT pivot motor. </p>
     *
     * <p> This list will be null if we are using a differential drive. </p>
     */
    private enum WheelIndex {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_RIGHT(2),
        BACK_LEFT(3);

        public final int label;

        private WheelIndex(int label) {
            this.label = label;
        }
    }

    // The differential drive does not care about the orders of the left and
    // right motors because the same gearbox is driving the entire right and
    // left side.
    public static final int DIFFERENTIAL_DRIVE_PWM_LEFT_MOTOR_1 = WheelIndex.FRONT_LEFT.label;
    public static final int DIFFERENTIAL_DRIVE_PWM_LEFT_MOTOR_2 = WheelIndex.BACK_LEFT.label;
    public static final int DIFFERENTIAL_DRIVE_PWM_RIGHT_MOTOR_1 = WheelIndex.FRONT_RIGHT.label;
    public static final int DIFFERENTIAL_DRIVE_PWM_RIGHT_MOTOR_2 = WheelIndex.BACK_RIGHT.label;

    /**
     * <p> Two motors cannot have the same ID on the CANBus; however, we still want
     * to use the WheelIndex enumeration as it provides us with an easy way to
     * know which motor is which. However, in a swerve drive, two motors are on
     * the same module, and that creates a conflict. </p>
     *
     * <p> To resolve the conflict, we will offset the IDs of all PivotMotors by the
     * number of drive motors. </p>
     *
     * <p> Suppose you have a random CAN ID. If it is between 0 and 3, it is a drive
     * motor. If it is between 4 and 7, it is a pivot motors. All other values
     * are neither. </p>
     *
     *
     * </p> To find the position of the CAN ID on the chassis, take CAN ID modulo 4. </p>
     */
    public static final int PIVOT_MOTOR_CAN_OFFSET = 4;
    public static final int DRIVE_MOTOR_CAN_OFFSET = 0;
  }
}
