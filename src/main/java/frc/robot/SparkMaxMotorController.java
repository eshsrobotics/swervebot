package frc.robot;

import com.revrobotics.spark.SparkMax;

public class SparkMaxMotorController implements frc.robot.abstractions.PtMotorController {

    private final SparkMax sparkMax;

    public SparkMaxMotorController(SparkMax sparkMax) {
        this.sparkMax = sparkMax;
    }

    @Override
    public void set(double speed) {
        sparkMax.set(speed);
    }

}
