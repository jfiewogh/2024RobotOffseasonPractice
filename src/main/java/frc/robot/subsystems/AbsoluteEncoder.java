package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

public class AbsoluteEncoder {

    public enum EncoderConfig {
       /* 

FL: 21.359872702898492 - 0.031389508928571425
FR: -0.011479935311237404 - -0.020926339285714284
BL: 0.012329594242832757 - -0.005231584821428571
BR: -0.04575296130634129 - -0.03662109375


forward

﻿﻿﻿﻿﻿﻿ FL: 26.6216925862155 - 5.247279575892857 ﻿
﻿﻿﻿﻿﻿﻿ FR: -3.7257622484188295 - -3.7458147321428568 ﻿
﻿﻿﻿﻿﻿﻿ BL: -6.654341244589345 - -6.691196986607142 ﻿
﻿﻿﻿﻿﻿﻿ BR: -1.3314681216653739 - -1.3602120535714286 ﻿
﻿﻿﻿﻿﻿﻿  ﻿


values facing forward

﻿﻿﻿﻿﻿﻿ FL: -6.136649333177951 - -5.634416852678571 ﻿
﻿﻿﻿﻿﻿﻿ FR: 19.60274977245161 - 9.892926897321429 ﻿
﻿﻿﻿﻿﻿﻿ BL: -2.6942661692716148 - 4.7607421875 ﻿
﻿﻿﻿﻿﻿﻿ BR: -1.0672433919882922 - -2.082170758928571


﻿﻿﻿﻿﻿﻿ FL: -0.12646484375 - -2.7099609375 ﻿
﻿﻿﻿﻿﻿﻿ FR: -0.06787109375 - -1.4543805803571428 ﻿
﻿﻿﻿﻿﻿﻿ BL: -0.09912109375 - -2.1240234375 ﻿
﻿﻿﻿﻿﻿﻿ BR: 0.0380859375 - 0.8161272321428571 ﻿


﻿﻿﻿﻿﻿﻿ FL: 0.240966796875 - 5.16357421875 ﻿
﻿﻿﻿﻿﻿﻿ FR: -0.04345703125 - -0.9312220982142857 ﻿
﻿﻿﻿﻿﻿﻿ BL: -0.2744140625 - -5.880301339285714 ﻿
﻿﻿﻿﻿﻿﻿ BR: 0.406982421875 - 8.721051897321429 ﻿



﻿﻿﻿﻿﻿﻿ FL: -4.712378520567725 - 0.5022321428571428 ﻿
﻿﻿﻿﻿﻿﻿ FR: -0.02092633832227199 - -9.636579241071429 ﻿
﻿﻿﻿﻿﻿﻿ BL: -0.5621280217321298 - 7.439313616071428 ﻿
﻿﻿﻿﻿﻿﻿ BR: -0.9264084581791959 - -0.9469168526785714 ﻿
﻿﻿﻿﻿﻿﻿  ﻿

last run
﻿﻿﻿﻿﻿﻿ FL: -0.490966796875 - -10.520717075892856 ﻿
﻿﻿﻿﻿﻿﻿ FR: 0.491455078125 - 10.531180245535714 ﻿
﻿﻿﻿﻿﻿﻿ BL: 0.484375 - 10.379464285714285 ﻿
﻿﻿﻿﻿﻿﻿ BR: 0.481689453125 - 10.321916852678571 ﻿



* 
         */

        

        // Modify these values
        FRONT_LEFT(23, 0.240966796875),
        FRONT_RIGHT(24, -0.06345703125),
        BACK_LEFT(25, -0.2744140625),
        BACK_RIGHT(26, -0.406982421875);

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
        
        magConfig.withMagnetOffset(offset);
        
        canConfig.withMagnetSensor(magConfig);
        encoder.getConfigurator().apply(canConfig);

        System.out.println(encoder.getAbsolutePosition().getValueAsDouble());
        
        return encoder;
    }
}
