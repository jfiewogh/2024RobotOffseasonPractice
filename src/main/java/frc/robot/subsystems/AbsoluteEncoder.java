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
         * FL: -8.379303932189941 ﻿
﻿﻿﻿﻿﻿﻿ FR: 4.319284439086914 ﻿
﻿﻿﻿﻿﻿﻿ BL: 12.496208190917969 ﻿
﻿﻿﻿﻿﻿﻿ BR: 0.07941550761461258 ﻿
        */
        /*
         * 0.02001953125 ﻿
﻿﻿﻿﻿﻿﻿ FR: 0.28466796875 ﻿
﻿﻿﻿﻿﻿﻿ BL: -0.364990234375 ﻿
﻿﻿﻿﻿﻿﻿ BR: 0.3037109375 ﻿
         */

         /*
          *  FL: 10.66341781616211 ﻿
﻿﻿﻿﻿﻿﻿ FR: -10.701981544494629 ﻿
﻿﻿﻿﻿﻿﻿ BL: 10.6759033203125 ﻿
﻿﻿﻿﻿﻿﻿ BR: -10.712526321411133
          */

          /*
           * ﻿﻿﻿﻿﻿﻿ FL: -10.635795593261719 ﻿
﻿﻿﻿﻿﻿﻿ FR: 53.5280647277832 ﻿
﻿﻿﻿﻿﻿﻿ BL: 53.625240325927734 ﻿
﻿﻿﻿﻿﻿﻿ BR: 53.517112731933594 ﻿
           */

           // immediately after running
           /*﻿﻿﻿﻿﻿﻿ FL: 2.5187160968780518 ﻿
﻿﻿﻿﻿﻿﻿ FR: -2.5304698944091797 ﻿
﻿﻿﻿﻿﻿﻿ BL: -2.7177951335906982 ﻿
﻿﻿﻿﻿﻿﻿ BR: -3.0458147525787354 ﻿
﻿﻿﻿﻿﻿﻿ FL: 10.78067684173584 ﻿
﻿﻿﻿﻿﻿﻿ FR: -10.721000671386719 ﻿
﻿﻿﻿﻿﻿﻿ BL: -10.598801612854004 ﻿
﻿﻿﻿﻿﻿﻿ BR: -10.56967830657959 ﻿

            // encoder values when going forward

            beginning
            FL: -12.222182137735453 ﻿
﻿﻿﻿﻿﻿﻿ FR: 6.99836846260003 ﻿
﻿﻿﻿﻿﻿﻿ BL: 13.131998548600873 ﻿
﻿﻿﻿﻿﻿﻿ BR: 11.127070283786667

            small adjust
            FL: -12.222182137735453 ﻿
﻿﻿﻿﻿﻿﻿ FR: 0.9507416142252926 ﻿
﻿﻿﻿﻿﻿﻿ BL: 13.25104419237475 ﻿
﻿﻿﻿﻿﻿﻿ BR: 11.127070283786667

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

            * 
            */

        // Modify these
        FRONT_LEFT(23, front_left_values[1] - front_left_values[0]),
        FRONT_RIGHT(24, front_right_values[1] - front_left_values[0]),
        BACK_LEFT(25, back_left_values[1] - front_left_values[0]),
        BACK_RIGHT(26, back_right_values[1] - front_left_values[0]);

        private int id;
        private double offset;

        // offset is in rotations
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
        
        return encoder;
    }
}
