package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.AbsoluteEncoderConstants;

public class AbsoluteEncoder {

    public enum EncoderConfig {
        /* 
        working angles
        * ﻿﻿﻿﻿﻿﻿ FL: 10.77183366287412 - 0.50244140625 - 10.7666015625 ﻿
        ﻿﻿﻿﻿﻿﻿ FR: 10.721383760716662 - 0.499267578125 - 10.698590959821429 ﻿
        ﻿﻿﻿﻿﻿﻿ BL: 10.775407217847869 - 0.50146484375 - 10.745675223214285 ﻿
        ﻿﻿﻿﻿﻿﻿ BR: 10.62027024737172 - 0.4970703125 - 10.651506696428571 ﻿
        */

        // Modify these values
        FRONT_LEFT(23, AbsoluteEncoderConstants.kFrontLeftOffset),
        FRONT_RIGHT(24, AbsoluteEncoderConstants.kFrontRightOffset),
        BACK_LEFT(25, AbsoluteEncoderConstants.kBackLeftOffset),
        BACK_RIGHT(26, AbsoluteEncoderConstants.kBackRightOffset);

        private int id;
        private double offset; // in wheel rotations // positive is counterclockwise

        private EncoderConfig(int id, double offset) {
            this.id = id;
            this.offset = offset;
        }

        public int getId() {
            return id;
        }

        public double getOffset() {
            return offset;
        }
    }

    public static CANcoder createAbsoluteEncoder(EncoderConfig config) {
        int id = config.getId();
        double offset = config.getOffset();

        CANcoder encoder = new CANcoder(id);
        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
        
        magConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        magConfig.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        magConfig.withMagnetOffset(offset);
        canConfig.withMagnetSensor(magConfig);
        encoder.getConfigurator().apply(canConfig);

        return encoder;
    }
}
