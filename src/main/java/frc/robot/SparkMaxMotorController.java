package frc.robot;

import com.revrobotics.spark.SparkMax;

public class SparkMaxMotorController implements frc.robot.abstractions.PtMotorController {

    private final SparkMax sparkMax;

    public SparkMaxMotorController(int deviceID, SparkMax.MotorType motorType) {
        sparkMax = new SparkMax(deviceID, motorType);
    }

    @Override
    public void set(double speed) {
        sparkMax.set(speed);
    }

}
