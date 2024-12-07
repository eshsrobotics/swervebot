package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

/**
 * An implementation of swerve modules similar to the 2020 code.
 * <ul>
 *  <li>Both drive and pivot motors will use the {@link RelativeEncoder}</li>
 *  <li> We will use as much of the 2020 robot code as possible for expedience.
 *
 */
public class RelativeEncoderBackend implements SwerveModuleBackend {

    private RelativeEncoder pivotEncoder, driveEncoder;

    /**
     * Create a {@link SwerveModuleBackend backend} that uses {@link
     * RelativeEncoder RelativeEncoders} for both driving and pivoting, which
     * is similar to the way Team 1759 used to handle its swerve modules from
     * 2020-2023.
     *
     * @param pivotEncoder The built-in brushless NEO encoder that measures the angle of the pivot
     *                     wheels.
     * @param driveEncoder The built-in brushless NEO encoder that measures
     *                     the angle of the drive wheels.
     */
    public RelativeEncoderBackend(RelativeEncoder pivotEncoder, RelativeEncoder driveEncoder) {
        this.pivotEncoder = pivotEncoder;
        this.driveEncoder = driveEncoder;

        // TODO: Configure the pivot encoder so its output corresponds to
        // actual degrees rotated by the swerve module.
        //
        // // TODO: Configure the drive encoder so its output corresponds to
        // the number of meters driven.
    }

    /**
     * {@inheritDoc}
     *
     * This implementation resets both the drive and pivot encoders.
     */
    @Override
    public void resetEncoder() {
        pivotEncoder.setPosition(0);
        driveEncoder.setPosition(0);
    }

    @Override
    public double getSpeedInMetersPerSecond() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSpeedInMetersPerSecond'");
    }

    @Override
    public double getDistanceInMeters() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDistanceInMeters'");
    }

    @Override
    public double getPivotAngleInDegrees() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivotAngleInDegrees'");
    }

    @Override
    public void setPivotAngleInDegrees(double PivotAngleInDegrees) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPivotAngleInDegrees'");
    }

}
