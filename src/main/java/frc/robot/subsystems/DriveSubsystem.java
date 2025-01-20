package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
    private List<PWMMotorController> differentialDriveMotors;


    public DriveSubsystem(InputSubsystem i, DriveType driveType) {
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
