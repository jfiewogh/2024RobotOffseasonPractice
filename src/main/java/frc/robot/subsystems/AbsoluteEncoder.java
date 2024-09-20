package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

public class AbsoluteEncoder {

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

            * 
            */

        // FRONT_LEFT(23, 0.26),
        // FRONT_RIGHT(24, 0.11),
        // BACK_LEFT(25, 0.3),
        // BACK_RIGHT(26, 0.25);
        FRONT_LEFT(23, -8.379),
        FRONT_RIGHT(24, 4.319),
        BACK_LEFT(25, 12.496),
        BACK_RIGHT(26, 0.0794);

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

        private double getOffset() {
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
