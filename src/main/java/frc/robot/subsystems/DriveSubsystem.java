package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants.DriveType;

public class DriveSubsystem extends SubsystemBase {
    private InputSubsystem input;
    private double maxDriveSpeed;
    private double maxTurnSpeed;
    private DriveType driveType;
    private DifferentialDrive differentialDrive;

    /**
     * <p> The list of PWM Spark Max motor controllers controlling the differential
     * drive motors. </p>
     *
     * <p> This array will be null if we are using a swerve drive. </p>
     */
    private List<PWMMotorController> differentialDriveMotors;

    /**
     * The list of Spark Maxes controlling the swerve drive motors.
     */
    private List<CANSparkMax> swerveDriveMotors;

    /**
     * The list of Spark Maxes controlling the swerve pivot motors.
     *
     * <p> This list will be null if we are using a differential drive. </p>
     */
    private List<CANSparkMax> swervePivotMotors;

    /**
     * To read each swerve module's pivot angle, we will use CANCoders (we will
     * ignore the built-in {@link RelativeEncoder}) because the CANCoders are
     * absolute encoders. This will ensure that each swerve module is reset to
     * the same position each time.
     *
     * <p> This list will be null if we are using a differential drive. </p>
     */
    private List<CANcoder> swerveCANCODER;
    /**
     * Intended to be owned by the RobotContainer and to be used by
     * RobotContainer's commands
     * @param inputSubsystem Provides human input for teleop regardless of the
     * drive type being used.
     * @param driveType the type of drive being used (swerve or differential)
     */
    public DriveSubsystem(InputSubsystem inputSubsystem, DriveType driveType) {
        super("swerveDrive");
        this.driveType = driveType;
        switch (driveType) {
            case DIFFERENTIAL_DRIVE:
                differentialDriveMotors = Arrays.asList(new PWMMotorController[] {
                    new PWMSparkMax(DriveConstants.DIFFERENTIAL_DRIVE_PWM_LEFT_MOTOR_1),
                    new PWMSparkMax(DriveConstants.DIFFERENTIAL_DRIVE_PWM_LEFT_MOTOR_2),
                    new PWMSparkMax(DriveConstants.DIFFERENTIAL_DRIVE_PWM_RIGHT_MOTOR_1),
                    new PWMSparkMax(DriveConstants.DIFFERENTIAL_DRIVE_PWM_RIGHT_MOTOR_2)
                });
                differentialDriveMotors.get(0).addFollower(differentialDriveMotors.get(1));
                differentialDriveMotors.get(2).addFollower(differentialDriveMotors.get(3));
                differentialDrive = new DifferentialDrive(differentialDriveMotors.get(0),
                                                          differentialDriveMotors.get(2));
                break;
            case SWERVE_DRIVE:
                break;
        }
    }

    public void periodic() {
        // ChassisSpeeds movement = new ChassisSpeeds(maxDriveSpeed * input.getForwardBack(), maxDriveSpeed * input.getLeftRight(),
                // maxTurnSpeed * input.getTurn());
        switch (driveType) {
            case DIFFERENTIAL_DRIVE:
                differentialDrive.arcadeDrive(input.getForwardBack(), input.getTurn());
                break;
            case SWERVE_DRIVE:
                break;
            default:
                break;
        }
    }

}
