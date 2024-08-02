package frc.robot.subsystems;

/**
 * Abstracts away the concept of grabbing numbers from a swerve module's
 * encoders. What numbers? The speed of the swerve modules. The distance that the
 * swerve modules traveled. The angle of the pivot motors.
 */
public interface SwerveModuleBackend {
    /**
     * Reset the encoder to 0. For the drive motor encoder, the distance
     * traveled will be reset to 0. For the pivot motor, the angle will
     * sometimes be reset to 0, depending on the backend.
     * <ul>
     *    <li>
     *     Useful for resetting the encoders without turning off the robot.
     *    </li>
     *    <li>
     *     Backends based on absolute angles will not reset to 0 after calling this method.
     *    </li>
     * </ul>
     */
    void resetEncoder();

    /**
     * Returns the speed of the swerve module drive motor.
     * @return a speed in meters per second
     */
    double getSpeedInMetersPerSecond();

    /**
     * Returns the distance that drive motors have traveled.
     * @return returns a distance in meters
     */
    double getDistanceInMeters();

    /**
     * Returns the angle of the swerve module pivot motor.
     * @return returns an angle in degrees
     */
    double getPivotAngleInDegrees();
};