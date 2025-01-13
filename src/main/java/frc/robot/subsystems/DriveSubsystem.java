package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private InputSubsystem input;
    private double maxDriveSpeed;
    private double maxTurnSpeed;

    public DriveSubsystem(InputSubsystem i, double d, double t) {
        super("swerveDrive");
        input = i;
        maxDriveSpeed = d;
        maxTurnSpeed = t;
    }

    public void periodic() {
        ChassisSpeeds movement = new ChassisSpeeds(maxDriveSpeed * input.getForwardBack(), maxDriveSpeed * input.getLeftRight(),
                maxTurnSpeed * input.getTurn());
    }

}
