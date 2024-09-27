// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kTau = Math.PI * 2;

  public static class SwerveConstants {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14);
    public static final double kP = 0.1;
    public static final double kAngleMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);
  } 

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AbsoluteEncoderConstants {
    // to move the wheel clockwise, increase
    // to move the wheel counterclockwise, decrease


    /*
    FL: 10.67766447874932 0.498291015625 10.677664620535714
FR: 10.756139405981244 0.501708984375 10.750906808035714
BL: 10.77183366287412 0.50244140625 10.7666015625
BR: 10.635811508027185 0.49658203125 10.641043526785714

FL: 4.606176971637067 0.21826171875 4.677036830357142
FR: 18.065561361790845 0.843505859375 18.07512555803571
BL: 14.343222404184024 0.67138671875 14.386858258928571
BR: 20.326151196337705 0.947998046875 20.314243861607142

    
     * 
     */
    
     // to see offset, you have to run twice, for some reason

    // set offset to 0, align the wheels then record absolute encoder values
    public static final double kFrontLeftOffset = -(0.21826171875 - 0.498291015625 + 0.5); // 0.84743377924 - 0.1;
    public static final double kFrontRightOffset = -(0.843505859375 - 0.501708984375 + 0.5); // 0.20410059132;
    public static final double kBackLeftOffset = -(0.67138671875 - 0.50244140625 + 0.5); // 0.29394180287;
    public static final double kBackRightOffset = -(0.947998046875 - 0.49658203125 + 0.5); // 0.07128637592;
  }
}
