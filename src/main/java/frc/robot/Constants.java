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
    public static final double kP = 0.05;
    public static final double kAngleMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);
  } 

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AbsoluteEncoderConstants {
    // to move the wheel clockwise, increase
    // to move the wheel counterclockwise, decrease

    // align the wheels then record absolute encoder values
    public static final double kFrontLeftOffset = 0.84743377924;
    public static final double kFrontRightOffset = 0.20410059132;
    public static final double kBackLeftOffset = 0.29394180287;
    public static final double kBackRightOffset = 0.57128637592;
  }
}
