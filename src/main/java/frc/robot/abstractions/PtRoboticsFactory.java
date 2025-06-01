package frc.robot.abstractions;

public interface PtRoboticsFactory {
    PtMotorController createBrushlessMotorController(int deviceID);

    PtMotorController createBrushedMotorController(int deviceID);
}