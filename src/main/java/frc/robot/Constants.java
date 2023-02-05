// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final class OI {
      public static final int driverPort = 0;
      public static final int opertatorPort = 1;
  }
  public final class Drivetrain {
      public static final double rotationGoal = 90;
      public static final int leftID          = 4;
      public static final int leftFollowerID  = 3;
      public static final int rightID         = 6;
      public static final int rightFollowerID = 5;

      // public static final double kP = 1.5;
      // public static final double kI = 0.015;
      // public static final double kD = 0.2;

      public static final double kP = 1.7;
      public static final double kI = 0;
      public static final double kD = 0.2;
      public static final double angle = 180;

      // public static final double kP2 = 1.5;
      // public static final double kI2 = 0.0;
      // public static final double kD2 = 0.2;
  }

  public final class Arm{
      public static final double kP = 2.0;
      public static final double kI = 0.0;
      public static final double kD = 0.2;

      public static final int ArmId = 7;
      public static final double angle =90;
  }
}
