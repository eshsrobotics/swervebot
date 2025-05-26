package frc.robot.factories;

public interface RoboticsFactory {
    PtMotorController createBrushlessMotorController(int deviceID);
}