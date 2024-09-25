package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

public class AbsoluteEncoder {

    static double[] front_left_values = {-96.43965800198558, -86.74947859544758};
    static double[] front_right_values = {-75.07546254825237, -78.45675225032663};
    static double[] back_left_values = {-75.03708720629164, -84.22846002269135};
    static double[] back_right_values = {-75.06586021297123, -75.01823709843826};


    public enum EncoderConfig {
        /*
            again (swerve)
            FL: -10.722187598090983 ﻿
﻿﻿﻿﻿﻿﻿ FR: 10.784129217635995 ﻿
﻿﻿﻿﻿﻿﻿ BL: 10.75106503077387 ﻿
﻿﻿﻿﻿﻿﻿ BR: 10.793731552917142

            // run
            ﻿﻿﻿﻿﻿﻿ FL: -96.39205917256967 ﻿
﻿﻿﻿﻿﻿﻿ FR: -75.02783943371942 ﻿
﻿﻿﻿﻿﻿﻿ BL: -75.03708720629164 ﻿
﻿﻿﻿﻿﻿﻿ BR: -75.04204622719304 ﻿

            // after running
            ﻿﻿﻿﻿﻿﻿ FL: -96.41585858727763 ﻿
﻿﻿﻿﻿﻿﻿ FR: -75.02783943371942 ﻿
﻿﻿﻿﻿﻿﻿ BL: -75.03708720629164 ﻿
﻿﻿﻿﻿﻿﻿ BR: -75.04204622719304 ﻿

            // arcade

            beginning
            ﻿﻿﻿﻿﻿﻿ FL: -96.43965800198558 ﻿
﻿﻿﻿﻿﻿﻿ FR: -75.07546254825237 ﻿
﻿﻿﻿﻿﻿﻿ BL: -75.03708720629164 ﻿
﻿﻿﻿﻿﻿﻿ BR: -75.06586021297123 ﻿

            // after align
            ﻿﻿﻿﻿﻿﻿ FL: -86.74947859544758 ﻿
﻿﻿﻿﻿﻿﻿ FR: -78.45675225032663 ﻿
﻿﻿﻿﻿﻿﻿ BL: -84.22846002269135 ﻿
﻿﻿﻿﻿﻿﻿ BR: -75.01823709843826 ﻿

        */

        // Modify these values
        FRONT_LEFT(23, 0), // front_left_values[1] - front_left_values[0]),
        FRONT_RIGHT(24, 0), // front_right_values[1] - front_right_values[0]),
        BACK_LEFT(25, 0), // back_left_values[1] - back_left_values[0]),
        BACK_RIGHT(26, 0); // back_right_values[1] - back_right_values[0]);

        private int id;
        private double offset; // in rotations

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
        
        magConfig.withMagnetOffset(offset);
        
        canConfig.withMagnetSensor(magConfig);
        encoder.getConfigurator().apply(canConfig);

        System.out.println(encoder.getAbsolutePosition().getValueAsDouble());
        
        return encoder;
    }
}
