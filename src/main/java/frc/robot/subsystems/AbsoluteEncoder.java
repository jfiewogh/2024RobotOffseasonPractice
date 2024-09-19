package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

public class AbsoluteEncoder {

    public enum EncoderConfig {
        FRONT_LEFT(23, -1.7441),
        FRONT_RIGHT(24, 2.0678),
        BACK_LEFT(25, -2.0801),
        BACK_RIGHT(26, 2.8041);

        private int id;
        private double offset;

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
        
        magConfig.withMagnetOffset(Units.radiansToRotations(offset));
        
        canConfig.withMagnetSensor(magConfig);
        encoder.getConfigurator().apply(canConfig);
        
        return encoder;
    }
}
