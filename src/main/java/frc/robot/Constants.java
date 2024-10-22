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
    public static final double kP = 0.05; // 0.1
    public static final double kAngleMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double kMaxAngleMotorSpeed = 1;
  } 

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AbsoluteEncoderConstants {
    // positive is clockwise, negative is counterclockwise
    /*
     * THESE SHOULD WORK, SO TRY NOT TO CHANGE
     * 
     * HOW TO GET THE VALUES
     * set the offsets to 0
     * run the robot on arcade drive, and align the wheels so that they are all facing forward
     * record the absolute encoder values
     * set the offsets to the negative of the recorded values
     * (To see the offsets, you have to run twice for some reason.)
     */

    /*
     * FL: 12.770298443785752 0.595947265625 12.770298549107142
      FR: 9.537179526362154 0.445068359375 9.537179129464285
      BL: 13.390508762563368 0.623779296875 13.36669921875
      BR: 7.214355436609842 0.33740234375 7.230050223214286

      FL: 9.841730679481872 0.732666015625 15.699986049107142
      FR: 12.037191472871049 0.328125 7.03125
      BL: 23.080827808524397 0.1689453125 3.620256696428571
      BR: 4.7143389366415 0.45458984375 9.7412109375

      // with offset

      FL: 21.503740164808082 0.857177734375 18.36809430803571
      FR: -0.041671614970108635 0.628662109375 13.471330915178571
      BL: 21.458917124270865 0.354248046875 7.591029575892857
      BR: 21.417695566592005 0.6318359375 13.539341517857142

      FL: 21.49232373128338 0.710693359375 15.229143415178571
      FR: 21.452624850443488 0.2548828125 5.461774553571428
      BL: -0.05184240697449231 0.71484375 15.318080357142856
      BR: 21.372549533998036 0.263671875 5.650111607142857
      
      FL: 21.47236865060427 0.417724609375 8.951241629464285
      FR: 0.043673401120231665 0.50732421875 10.871233258928571
      BL: 21.368484205404044 0.429443359375 9.202357700892856
      BR: -0.04324022008805822 0.53515625 11.467633928571429

      0 offset

      FL: -0.04879039386211519 0.108642578125 2.328055245535714
      FR: 21.371080284416575 0.691162109375 14.810616629464285
      BL: 0.05946527754318043 0.6884765625 14.753069196428571
      BR: 21.467472771007206 0.609619140625 13.063267299107142

      FL: -0.02908675663660742 0.21728515625 4.656110491071428
      FR: 21.376707146036814 0.38330078125 8.213588169642858
      BL: 21.4196286619091 0.372802734375 7.988630022321428
      BR: 21.377887402725392 0.22216796875 4.7607421875

      unary minus

      FL: -0.050396593479621395 0.8505859375 18.226841517857142
      FR: 21.49457253312211 0.5400390625 11.572265625
      BL: 21.4838967956674 0.518310546875 11.106654575892856
      BR: -0.0570277680126929 0.88232421875 18.906947544642858
     
      FL: 21.412034705807898 0.7001953125 15.004185267857142
      FR: 21.357820610507858 0.08203125 1.7578125
      BL: 21.42125090772797 0.03515625 0.7533482142857143
      BR: 21.42025278941726 0.766357421875 16.421944754464285

      for some reason angle is doubled every run
      */

       /* 2024 Robot Code Values
    FrontLeftModule(Constants.canIdFrontLeftCancoder, false, -1.7441, -1.1520),
    FrontRightModule(Constants.canIdFrontRightCancoder, false, 2.0678, 2.0816 - (Math.PI)),
    BackLeftModule(Constants.canIdBackLeftCancoder, false, -2.0801 + (Math.PI), -0.9664),
    BackRightModule(Constants.canIdBackRightCancoder, false, 2.8041, -0.5906);
    */

    public static final double kFrontLeftOffset = -1.7441 / Constants.kTau; // (0.732666015625);
    public static final double kFrontRightOffset = 2.0678 / Constants.kTau; // (0.328125);
    public static final double kBackLeftOffset = (-2.0801 + Math.PI) / Constants.kTau; // (0.1689453125);
    public static final double kBackRightOffset = 2.8041 / Constants.kTau; // (0.45458984375
  }

  public static class MotorConstants {
    public static final int kIntakeDeployMotorDeviceId = 16;
    public static final int kIntakeRollerMotorDeviceId = 9;
    public static final int kIntakeIndexMotorDeviceId = 15; 
  }

  public static class IntakeConstants {
    public static final double kDeployPosition = 12;
    public static final double kRetractPosition = 0;
    public static final double kP = 0.04;
  }
}
