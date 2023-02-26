package frc.robot;

public final class Constants {
  public static final class OI {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double DEADBAND_VALUE = 0.1;
  }

  public static final class ArcadeDrive {
    public static final double FORWARD_MULTIPLIER = 1;
    public static final double FORWARD_SENSITIVE_MULTIPLIER = 0.7;

    public static final double ROTATION_MULTIPLIER = 0.7;
    public static final double ROTATION_SENSITIVE_MULTIPLIER = 0.7;
  }

  public static final class Drivetrain {
    public static final int LEFT_ID = 4;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_ID = 5;
    public static final int RIGHT_FOLLOWER_ID = 6;

    public static final int LEFT_ENCODER_CHANNEL_A = 3;
    public static final int LEFT_ENCODER_CHANNEL_B = 2;
    public static final int RIGHT_ENCODER_CHANNEL_A = 6;
    public static final int RIGHT_ENCODER_CHANNEL_B = 7;

    public static final double VELOCITY_KP = 0.014;
    public static final double VELOCITY_KI = 0;
    public static final double VELOCITY_KD = 0.008;

    public static final double VOLTAGE_KP = 0;
    public static final double VOLTAGE_KI = 0;
    public static final double VOLTAGE_KD = 0;

    public static final double MAX_VELOCITY = 4.6; // meters per second

  }

  public static final class Intake {
    public static final int MOTOR_ID = 7;
  }
  
  public static final class Arm {
    public static final int MOTOR_ID = 9;
    public static final int ENCODER_ID = 8;

    public static final double KP_REST = 2.7;
    public static final double KI_REST = 0;
    public static final double KD_REST = 0.3;

    public static final double KP_FIRST = 1.6;
    public static final double KI_FIRST = 0.01;
    public static final double KD_FIRST = 0.3;

    public static final double KP_SECOND = 1.9;
    public static final double KI_SECOND = 0.015;
    public static final double KD_SECOND = 0.5;

    public static final double KP_THIRD = 2.4;
    public static final double KI_THIRD = 0.018;
    public static final double KD_THIRD = 0.05;

    public static final double KP_FEEDER = 0;
    public static final double KI_FEEDER = 0;
    public static final double KD_FEEDER = 0;

    public static final double TOLERANCE_POSITION = 3.0 / 360;
    public static final double TOLERANCE_VELOCITY = 10 ;

    public static final double POSITION_FEEDER = 0;
    public static final double POSITION_THIRD_LEVEL = 100;
    public static final double POSITION_SECOND_LEVEL = 80;
    public static final double POSITION_FIRST_LEVEL = 40;
    public static final double POSITION_REST = 0;

    // feed forward 
    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double KPFeedForward = 0; 
    public static final double KIFeedForward = 0;
    public static final double KDFeedForward = 0;
    public static final double  MAX_SPEED= 0;

    public static final double multiplierController = 0.2;

    public static final int LIMIT_SWITCH_ID = 9;


  }

  public static final class chargeStationBalance {
    public static final boolean IS_REVERSED = true;
  }

  public static final class GetOnChargeStationAuto {
    public static final double DRIVE_SPEED = 0.6;
    public static final double FINISH_ANGLE = 12;
  }

  public static final class BalanceOnChargeStationDistance {
    public static final double DISTANCE = 1;
  }

  public static final class BalanceOnChargeStationAuto {
    public static final double DRIVE_SPEED_FORWARD = 0.3;
    public static final double DRIVE_SPEED_BACKWARDS = 0.22;
    public static final double FINISH_ANGLE = 0.5;
    public static final double MISTAKE_ANGLE = 10;
  }

  public static final class BalanceOnChargeStationPID {
    public static final double KP = 0.2;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double FINISH_ANGLE = 0;
  }

  public static final class DriveToDistance {
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
  }
}
