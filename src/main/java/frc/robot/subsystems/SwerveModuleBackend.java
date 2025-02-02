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
     * @return a number between 1 (full speed forward) and -1 (full speed
     *         backward). 0 means stopped.
     */
    double getDriveSpeed();

    /**
     * Sets the speed of this swerve module's drive motor.
     * @param driveSpeedPercent A value between 1 (full speed forward) and -1
     *                          (full speed backwards). 0 stops the drive motor.
     */
    void setDriveSpeed(double driveSpeedPercent);

    /**
     * Returns the number of rotations of swerve module drive motor since the
     * last reset.
     * @return returns a distance in revolutions
     */
    double getRevolutions();


    /**
     * Returns the angle of the swerve module pivot motor.
     * @return returns an angle in degrees
     */
    double getPivotAngle();

    /**
     * Sets angles for the swerve module pivot motor.
     * @param PivotAngleInDegrees The angle you want the swerve module to be.
     *                            The function will perform modulus operations
     *                            if necessary.
     */
    void setPivotAngle(double PivotAngleInDegrees);

};