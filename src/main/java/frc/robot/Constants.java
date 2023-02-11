package frc.robot;

public final class Constants {
  public final class OI {
    public static final int DRIVER_PORT = 0;
    public static final int OPERTATOR_PORT = 1;
  }

  public final class Drivetrain {
    public static final int LEFT_ID = 4;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_ID = 6;
    public static final int RIGHT_FOLLOWER_ID = 5;

    public static final int LEFT_ENCODER_CHANNEL_A = 0;
    public static final int LEFT_ENCODER_CHANNEL_B = 1;
    public static final int RIGHT_ENCODER_CHANNEL_A = 3;
    public static final int RIGHT_ENCODER_CHANNEL_B = 2;
  }

  public final class ArcadeDrive {
    public static final double KP = 1;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double MAX_SPEED = 4.6; // meters per second
  }
}
