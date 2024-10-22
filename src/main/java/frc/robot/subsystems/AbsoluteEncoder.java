package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.AbsoluteEncoderConstants;

// Copied from 2024 Robot Code

public class AbsoluteEncoder {

    public enum EncoderConfig {
        // Modify these values
        FRONT_LEFT(23, AbsoluteEncoderConstants.kFrontLeftOffset),
        FRONT_RIGHT(24, AbsoluteEncoderConstants.kFrontRightOffset),
        BACK_LEFT(25, AbsoluteEncoderConstants.kBackLeftOffset),
        BACK_RIGHT(26, AbsoluteEncoderConstants.kBackRightOffset);

        private int deviceId;
        private double offset; // in wheel rotations // positive is counterclockwise

        private EncoderConfig(int deviceId, double offset) {
            this.deviceId = deviceId;
            this.offset = offset;
        }

        public int getDeviceId() {
            return deviceId;
        }

        public double getOffset() {
            return offset;
        }
    }

    public static CANcoder createAbsoluteEncoder(EncoderConfig config) {
        int deviceId = config.getDeviceId();
        double offset = config.getOffset();

        CANcoder encoder = new CANcoder(deviceId);
        CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        
        magnetSensorConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        magnetSensorConfigs.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        magnetSensorConfigs.withMagnetOffset(offset);

        CANcoderConfig.withMagnetSensor(magnetSensorConfigs);
        encoder.getConfigurator().apply(CANcoderConfig);

        return encoder;
    }
}
