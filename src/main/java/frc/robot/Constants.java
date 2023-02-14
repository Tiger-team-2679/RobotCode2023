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

  public final class Intake {
    public static final int MOTOR_ID = 8;
  }
  
  public final class Arm {
    public static final int MOTOR_ID = 9;
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double TOLERANCE_POSTION = 0;
    public static final double TOLERANCE_VELOCITY = 0;

    public static final double POSTION_FEEDER = 120;
    public static final double POSTION_SECOND_LEVEL = 70;
    public static final double POSTION_FIRST_LEVEL = 30;
    public static final double POSTION_REST = 0;

    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double KPF = 0;
    public static final double KIF = 0;
    public static final double KDF = 0;
    public static final double  MAX_SPEED= 0;


  }
}
