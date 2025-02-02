package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

/**
 * An implementation of swerve modules using CANCoders for the pivot motors and
 * built-in NEO Relative Encoders for the drive motors.
 */
public class CANCoderBackend implements SwerveModuleBackend {

    /** Owned by CANCoderBackend */
    private CANcoder pivotEncoder;
    /** Passed into CANCoderBackend */
    private CANSparkMax driveMotorController, pivotMotorController;

    /**
     * Creates a CANCoderBackend that owns a CAN coder to control pivot angles.
     * Driving and turning are done with the two provided motor controller
     * objects.
     *
     * <p>Caller needs to provide the ID of the CAN coder on the CAN bus.</p>
     *
     * @param driveMotor The motor this backend object will use for driving.
     * @param pivotMotor The motor this backend object will use for pivoting.
     * @param canID The ID of the CAN coder on the CAN bus.
     */
    public CANCoderBackend(CANSparkMax driveMotor, CANSparkMax pivotMotor, int canID) {
        driveMotorController = driveMotor;
        pivotMotorController = pivotMotor;
        pivotEncoder = new CANcoder(canID);

        // - A conversion factor of 1.0 will bypass conversion (i.e.,
        //   getVelocity() would return values in units of RPM.)
        //
        //   So how do we convert revolutions per minute to meters per second?
        //
        //   X revolutions    1 minute      (2 * PI * radius) meters        (2 * PI * radius)
        //   ------------- * ----------- * -------------------------- = X * ----------------- meters per second
        //      1 minute      60 seconds          1 revolution                      60

        double conversionFactor = (2 * Math.PI * Constants.DriveConstants.SWERVE_MODULE_WHEEL_RADIUS_METERS) / 60;
        driveMotorController.getEncoder().setVelocityConversionFactor(conversionFactor);
    }

    @Override
    public void resetEncoder() {

        // TODO: This value of 0 is *almost certainly* not correct.  It's likely
        // the case that each swerve module will have a slightly different
        // number which represents its "straight forward" position, since the
        // magnets are all positioned slightly differently.
        //
        // Realistically, we will need an array of four angle value constants,
        // one for each swerve module.
        pivotEncoder.setPosition(0);

        // Turns out that we can't tell driveMotorController.getEncoder() to
        // reset.  So we'll just leave it -- who cares.
        // driveMotorController.getEncoder().reset(); // <--- Does not exist.
    }

    @Override
    public double getDriveSpeed() {
        return driveMotorController.getEncoder().getVelocity();
    }

    /**
     * Returns the distance the drive motors have traveled. However, in the
     * current implementation, if the robot drives backwards (we do not know
     * what is "backwards"), the distance will decrease and may become negative.
     */
    @Override
    public double getRevolutions() {
        return driveMotorController.getEncoder().getPosition() * 2 * Math.PI
            * Constants.DriveConstants.SWERVE_MODULE_WHEEL_RADIUS_METERS;
    }

    /**
     * Returns the absolute angle of the swerve module in degrees.  The output
     * range of this function is the half-open interval [0, 360).
     */
    @Override
    public double getPivotAngle() {
        return (pivotEncoder.getPosition().getValueAsDouble() % 360 + 360) % 360;
    }

    /**
     * Rotate the pivot motor so that is has the given absolute angle.
     *
     * @param pivotAngleInDegrees The desired absolute position, with 0 degrees
     *                            pointing the swerve module straight forward.
     */
    @Override
    public void setPivotAngle(double pivotAngleInDegrees) {

        // PROBLEM:
        //
        // - pivotMotorController.encoder() is a relative encoder.
        // - pivotEncoder is an absolute encoder, but does not influence the
        //   drive at all.
        //
        // QUESTION:
        //
        // - How do we get the pivotMotorController to rotate to an absolute
        //   bearing of X degrees?
        //
        // ANSWER:
        //
        // - We have to find the delta between the motor controller's encoder
        //   and the pivotEncoder.
        final double absoluteEncoderPosition = pivotEncoder.getPosition().getValueAsDouble();
        final double sourceAngleDegrees = absoluteEncoderPosition * 360;
        final double degreesToRotate = signedDelta(sourceAngleDegrees, pivotAngleInDegrees);

        var pivotPidController = pivotMotorController.getPIDController();

        // R = relative encoder's (CANSparkMax) current position
        // A = absolute encoder's current position (CANCoder)
        // D = degreesToRotate
        //
        // I think we need to go from R to R + D.
        // The rotation that gets us from R to R + D is (R + D - R) = D.
        final double relativeEncoderPosition = pivotMotorController.getEncoder().getPosition();
        pivotPidController.setReference(relativeEncoderPosition + degreesToRotate,
                                        CANSparkMax.ControlType.kPosition);

        // throw new UnsupportedOperationException("Unimplemented method 'setPivotAngleInDegrees'");
    }

    /**
     * You have two angles, source and target.  What is the signed difference
     * between them -- the smallest signed angle you would need to add to the
     * source in order to rotate it to the target?
     *
     * <p>That is what this function returns.</p>
     *
     * @param sourceAngleDegrees Your current bearing.
     * @param targetAngleDegrees Your desired bearing.
     * @return Returns the minimum number of degrees to add to your current
     *         bearing to get to your desired bearing.
     */
    private double signedDelta(double sourceAngleDegrees, double targetAngleDegrees) {
        double delta = targetAngleDegrees - sourceAngleDegrees;
        double signedDelta = (delta + 180) % 360 - 180;
        return signedDelta;
    }

    @Override
    public void setDriveSpeed(double driveSpeedPercent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveSpeed'");
    }
}
