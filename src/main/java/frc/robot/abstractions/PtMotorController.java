package frc.robot.abstractions;

public interface PtMotorController {
    void set(double speed);
    void stopMotor();
    double get();
    void disable();
}