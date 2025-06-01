package frc.robot;

import frc.robot.abstractions.PtMotorController;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FrcRoboticsFactory implements frc.robot.abstractions.PtRoboticsFactory {

    @Override
    public frc.robot.abstractions.PtMotorController createBrushlessMotorController(int deviceID) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        PtMotorController motorController = new SparkMaxMotorController(deviceID, SparkMax.MotorType.kBrushless);
        motorController.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return
    }

    @Override
    public PtMotorController createBrushedMotorController(int deviceID) {
        return new SparkMaxMotorController(deviceID, SparkMax.MotorType.kBrushed);
    }


}
