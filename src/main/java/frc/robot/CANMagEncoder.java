package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.abstractions.PtMagEncoder;

public class CANMagEncoder implements PtMagEncoder {

    private final CANcoder cancoder;

    public CANMagEncoder(CANcoder cancoder) {
        this.cancoder = cancoder;
    }

    @Override
    public double getAbsolutePositionAsDouble() {
        return cancoder.getAbsolutePosition(true).getValueAsDouble() * 2 * Math.PI;
    }
}
