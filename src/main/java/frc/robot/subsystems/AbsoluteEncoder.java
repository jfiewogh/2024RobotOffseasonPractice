package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.AbsoluteEncoderConstants;

// Copied from 2024 Robot Code

/*
 * ﻿﻿﻿﻿﻿﻿ FL: 0.11622327679959245 0.330078125 7.073102678571428 ﻿
﻿﻿﻿﻿﻿﻿ FR: -0.09422615928612632 -0.32470703125 -6.9580078125 ﻿
﻿﻿﻿﻿﻿﻿ BL: -0.07631534473922803 -0.08740234375 -1.8729073660714284 ﻿
﻿﻿﻿﻿﻿﻿ BR: 0.1093774635718533 -0.288330078125 -6.178501674107142 ﻿

﻿﻿﻿﻿﻿﻿ FL: -0.08355683462013558 -0.34228515625 -7.334681919642857 ﻿
﻿﻿﻿﻿﻿﻿ FR: 0.04800617905209849 0.355224609375 7.611955915178571 ﻿
﻿﻿﻿﻿﻿﻿ BL: 0.0738466487547177 -0.17041015625 -3.651646205357143 ﻿
﻿﻿﻿﻿﻿﻿ BR: 0.1018433570032538 0.42529296875 9.113420758928571 ﻿

 */

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
        
        magnetSensorConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        magnetSensorConfigs.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        // don't flip offset because flipped joystick I think
        magnetSensorConfigs.withMagnetOffset(offset);

        CANcoderConfig.withMagnetSensor(magnetSensorConfigs);
        encoder.getConfigurator().apply(CANcoderConfig);

        return encoder;
    }
}
