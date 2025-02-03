package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private List<CANSparkMax> differentialDriveMotors;

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
     * Our CANCoders measure absolute angles that don't change even when the
     * robot is power-cycled. However, the CANCoder's zero position is not
     * necessarily the same as the actual swerve module zero position. I mean,
     * maybe it is, maybe it isn't!
     *
     * Where we notice a discrepancy, we will record the actual angle the pivot
     * motor needs in order for the swerve module to face directly forward
     * (angle = zero degrees.)
     */
    private double[] CAN_CODER_ANGLE_OFFSETS = { // TODO: These numbers are made-up.
        -61,  // FRONT_LEFT
        -59,  // FRONT_RIGHT
        0,    // BACK_RIGHT
        12    // BACK_LEFT
    };

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
            case DIFFERENTIAL_DRIVE: {
                final int FRONT_LEFT = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.FRONT_LEFT.label;
                final int FRONT_RIGHT = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.FRONT_RIGHT.label;
                final int BACK_RIGHT = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.BACK_RIGHT.label;
                final int BACK_LEFT = DriveConstants.DRIVE_MOTOR_CAN_OFFSET + DriveConstants.WheelIndex.BACK_LEFT.label;
                differentialDriveMotors = Arrays.asList(new CANSparkMax[] {
                    new CANSparkMax(FRONT_LEFT, MotorType.kBrushed),
                    new CANSparkMax(FRONT_RIGHT, MotorType.kBrushed),
                    new CANSparkMax(BACK_RIGHT, MotorType.kBrushed),
                    new CANSparkMax(BACK_LEFT, MotorType.kBrushed)
                });
                differentialDriveMotors.get(FRONT_LEFT).addFollower(differentialDriveMotors.get(BACK_LEFT));
                differentialDriveMotors.get(FRONT_RIGHT).addFollower(differentialDriveMotors.get(BACK_RIGHT));
                differentialDrive = new DifferentialDrive(differentialDriveMotors.get(FRONT_LEFT),
                                                          differentialDriveMotors.get(FRONT_RIGHT));
                break;
            }
            case SWERVE_DRIVE: {
                final int FRONT_LEFT = DriveConstants.WheelIndex.FRONT_LEFT.label;
                final int FRONT_RIGHT = DriveConstants.WheelIndex.FRONT_RIGHT.label;
                final int BACK_RIGHT = DriveConstants.WheelIndex.BACK_RIGHT.label;
                final int BACK_LEFT = DriveConstants.WheelIndex.BACK_LEFT.label;

                // Initialize drive motors
                swerveDriveMotors = Arrays.asList(new CANSparkMax[] {
                    new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + FRONT_LEFT, MotorType.kBrushless),
                    new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + FRONT_RIGHT, MotorType.kBrushless),
                    new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + BACK_RIGHT, MotorType.kBrushless),
                    new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_CAN_OFFSET + BACK_LEFT, MotorType.kBrushless)
                });

                // Initialize pivot motors
                swervePivotMotors = Arrays.asList(new CANSparkMax[] {
                    new CANSparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + FRONT_LEFT, MotorType.kBrushless),
                    new CANSparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + FRONT_RIGHT, MotorType.kBrushless),
                    new CANSparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + BACK_RIGHT, MotorType.kBrushless),
                    new CANSparkMax(Constants.DriveConstants.PIVOT_MOTOR_CAN_OFFSET + BACK_LEFT, MotorType.kBrushless)
                });

                // Initialize the pivot motors in a similar manner to how we
                // initialized them for the 2020bot.
                swervePivotMotors.forEach(motor -> {
                    motor.setIdleMode(IdleMode.kBrake);
                });

                // Initialize CANcoders
                swerveCANCODER = Arrays.asList(new CANcoder[] {
                    new CANcoder(Constants.DriveConstants.CAN_CODER_CAN_OFFSET + FRONT_LEFT),
                    new CANcoder(Constants.DriveConstants.CAN_CODER_CAN_OFFSET + FRONT_RIGHT),
                    new CANcoder(Constants.DriveConstants.CAN_CODER_CAN_OFFSET + BACK_RIGHT),
                    new CANcoder(Constants.DriveConstants.CAN_CODER_CAN_OFFSET + BACK_LEFT)
                });
                break;
            }
        }
    }

    /**
     * Update the position of the drive according to human input (during teleop)
     * or according to the current trajectory (during autonomous).
     */
    public void periodic() {
        switch (driveType) {
            case DIFFERENTIAL_DRIVE:
                differentialDrive.arcadeDrive(input.getForwardBack(), input.getTurn());
                break;
            case SWERVE_DRIVE:
                // Convert the human input into a ChassisSpeeds object giving us
                // the overall bearing of the chassis.
                ChassisSpeeds movement =
                    new ChassisSpeeds(Constants.DriveConstants.SWERVE_DRIVE_MAX_DRIVING_SPEED * input.getForwardBack(),
                                      Constants.DriveConstants.SWERVE_DRIVE_MAX_DRIVING_SPEED * input.getLeftRight(),
                                      Constants.DriveConstants.SWERVE_DRIVE_MAX_TURNING_SPEED * input.getTurn());

                // With inverse kinematics, convert the overall chassis speed
                // into the speeds and angles for all four swerve modules.
                final int FRONT_LEFT = DriveConstants.WheelIndex.FRONT_LEFT.label;
                final int FRONT_RIGHT = DriveConstants.WheelIndex.FRONT_RIGHT.label;
                final int BACK_RIGHT = DriveConstants.WheelIndex.BACK_RIGHT.label;
                final int BACK_LEFT = DriveConstants.WheelIndex.BACK_LEFT.label;
                SwerveDriveKinematics kinematics =
                    new SwerveDriveKinematics(Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(FRONT_LEFT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(FRONT_RIGHT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(BACK_RIGHT),
                                              Constants.DriveConstants.SWERVE_MODULE_POSITIONS.get(BACK_LEFT));

                SwerveModuleState[] swerveStates = kinematics.toSwerveModuleStates(movement);

                // Scenario: FRONT_LEFT swerve module
                // - It is currently facing at 90 degrees (so it points right.)
                // - According to human input, we need it to point at -45
                //   degrees (45 degrees left of center).
                // - If we ASK the swerve module "what is your angle", the
                //   RelativeEncoder will tell us "why, 21.82 degrees, of
                //   course."  Because since the robot turned on, the module has
                //   rotated 21.82 degrees.
                // - PROBLEM: 21.82 does not equal 90.
                //   * The swerve module is unaware of its own absolute angle.
                //   * Only one entity holds that source of truth: the CANCoder.

                // Solution.SEPEHR:
                // 1. Have a PID controller

                break;
            default:
                break;
        }
    }

}
