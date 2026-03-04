package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {
    private InputSubsystem input;

    private SparkMax lowerIntake;

    private SparkMax thresholdIntake;
    
    private boolean testLowerIntake = false;
    private double lowerIntakeSpeed = 0;
    private boolean testThresholdIntake = false;
    private double thresholdSpeed = 0;

    public IntakeSubsystem(InputSubsystem inputSubsystem) {
        input = inputSubsystem;
        lowerIntake = new SparkMax(5, MotorType.kBrushed);
        thresholdIntake = new SparkMax(7, MotorType.kBrushed);
        SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("testLowerIntake", () -> testLowerIntake, (value) -> { testLowerIntake = value; });
        builder.addDoubleProperty("lowerIntakeTestValue", () -> lowerIntakeSpeed, (value) -> { lowerIntakeSpeed = value; });
        builder.addBooleanProperty("testThresholdIntake", () -> testThresholdIntake, (value) -> { testThresholdIntake = value; });
        builder.addDoubleProperty("thresholdIntakeTestValue", () -> thresholdSpeed, (value) -> { thresholdSpeed = value; });
    }

    @Override
    public void periodic() {
        if (testLowerIntake || input.getLowerIntakeButton() != 0) {
            lowerIntakeSpeed = input.getLowerIntakeButton() == 1 ? -0.3 : -0.6;
            lowerIntake.set(lowerIntakeSpeed);
        } else {
            lowerIntake.set(0.0);
        }
        if (testThresholdIntake || input.getThresholdIntakeButton() != 0) {
            thresholdSpeed = input.getThresholdIntakeButton() == 1 ? 0.4 : -0.4;
            thresholdIntake.set(thresholdSpeed);
        } else {
            thresholdIntake.set(0.0);
        }
    }
}
