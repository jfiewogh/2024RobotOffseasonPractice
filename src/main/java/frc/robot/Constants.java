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
    public static final double kP = 0.1; // 0.1
    public static final double kAngleMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double kMaxAngleMotorSpeed = 1;
  } 

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AbsoluteEncoderConstants {
    // positive is clockwise, negative is counterclockwise
    /*
     * HOW TO GET THE VALUES
     * set the offsets to 0
     * run the robot on arcade drive, and align the wheels so that they are all facing forward
     * record the absolute encoder values
     * set the offsets to the negative of the recorded values
     * (To see the offsets, you have to run twice for some reason.)
     */
    public static final double kFrontLeftOffset = -(0.21826171875);
    public static final double kFrontRightOffset = -(0.843505859375);
    public static final double kBackLeftOffset = -(0.67138671875);
    public static final double kBackRightOffset = -(0.947998046875);
  }

  public static class MotorConstants {
    public static final int kIntakeDeployMotorDeviceId = 16;
    public static final int kIntakeRollerMotorDeviceId = 9;
    public static final int kIntakeIndexMotorDeviceId = 15; 
  }

  public static class IntakeConstants {
    public static final double kDeployPosition = 0;
    public static final double kRetractPosition = 0;
    public static final double kP = 0.1;
  }
}
