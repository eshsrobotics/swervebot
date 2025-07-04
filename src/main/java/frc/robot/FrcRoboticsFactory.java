package frc.robot;

import frc.robot.abstractions.PtMotorController;
import frc.robot.abstractions.PtMagEncoder;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FrcRoboticsFactory implements frc.robot.abstractions.PtRoboticsFactory {

    @Override
    public frc.robot.abstractions.PtMotorController createBrushlessMotorController(int deviceID) {
        return createMotorController(deviceID, SparkMax.MotorType.kBrushless);
    }

    @Override
    public PtMotorController createBrushedMotorController(int deviceID) {
        return createMotorController(deviceID, SparkMax.MotorType.kBrushed);
    }

    private frc.robot.abstractions.PtMotorController createMotorController(int deviceID, SparkMax.MotorType motorType) {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        var sparkMax = new SparkMax(deviceID, motorType);
        sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        var motorController = new SparkMaxMotorController(sparkMax);
        return motorController;
    }

    @Override
    public frc.robot.abstractions.PtMagEncoder createMagEncoder(int deviceID) {
        var cancoder = new CANcoder(deviceID);
        var magEncoder = new CANMagEncoder(cancoder);
        return magEncoder;
    }

}
