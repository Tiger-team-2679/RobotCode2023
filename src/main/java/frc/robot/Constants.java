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

    public static final double VELOCITY_KP = 0.014;
    public static final double VELOCITY_KI = 0;
    public static final double VELOCITY_KD = 0.008;
    public static final double MAX_VELOCITY = 4.6; // meters per second

  }

  public final class Intake {
    public static final int MOTOR_ID = 7;
  }
  
  public final class Arm {
    public static final int MOTOR_ID = 9;
    public static final int ENCODER_ID = 4;

    public static final double KP = 1;
    public static final double KI = 0;
    public static final double KD = 0.2;

    public static final double TOLERANCE_POSTION = 0.05;
    public static final double TOLERANCE_VELOCITY = 0;

    public static final double POSTION_FEEDER = 120;
    public static final double POSTION_SECOND_LEVEL = 70;
    public static final double POSTION_FIRST_LEVEL = 30;
    public static final double POSTION_REST = 0;

    // feed forward 
    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double KPFeedForward = 0; 
    public static final double KIFeedForrward = 0;
    public static final double KDFeedForward = 0;
    public static final double  MAX_SPEED= 0;

    public static final double multiplierController = 0.2;

    public static final int LIMITSWITCH_ID = 9;


  }

  public final class GetOnChargeStationAuto {
    public static final double DRIVE_SPEED = 0.4;
    public static final double FINISH_ANGLE = 10;
  }

  public final class BalanceOnChargeStationAuto {
    public static final double DRIVE_SPEED_FORWARD = 0.2;
    public static final double DRIVE_SPEED_BACKWARDS = -0.2;
    public static final double FINISH_ANGLE = 0.5;
    public static final double MISTAKE_ANGLE = 5;
  }

  public final class BalanceOnChargeStationPID {
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double FINISH_ANGLE = 0;
  }

  public final class DriveToDistance {
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double FINISH_POINT = 0;
  }
}
