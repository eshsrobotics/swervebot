package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

public class CANMagEncoder implements frc.robot.abstractions.PtMagEncoder {

    private final CANcoder cancoder;

    public CANMagEncoder(CANcoder cancoder) {
        this.cancoder = cancoder;
    }

    @Override
    public double getAbsolutePositionAsDouble() {
        return cancoder.getAbsolutePosition(true).getValueAsDouble() * 2 * Math.PI;
    }
}
