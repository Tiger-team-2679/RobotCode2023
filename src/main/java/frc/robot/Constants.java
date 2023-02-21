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
    public static final int RIGHT_ENCODER_CHANNEL_A = 2;
    public static final int RIGHT_ENCODER_CHANNEL_B = 3;
  }

  public final class ArcadeDrive {
    public static final double KP = 0;
    // public static final double KP = 0.08;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double MAX_SPEED = 4.6; // meters per second
  }

  public final class Intake {
    public static final int MOTOR_ID = 8;
  }

  public final class GetOnChargeStationAuto {
    public static final double DRIVE_SPEED = 0.4;
    public static final double FINISH_ANGLE = 10;
  }

  public final class BalanceOnChargeStationAuto {
    public static final double DRIVE_SPEED_FORWARD = 0.2;
    public static final double FINISH_ANGLE = 0.5;
  }
}
