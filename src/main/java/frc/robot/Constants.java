package frc.robot;

public final class Constants {
  public static final class OI {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double JOYSTICKS_DEADBAND_VALUE = 0.1;
  }

  public static final class ArcadeDrive {
    public static final double FORWARD_MULTIPLIER = 0.7;
    public static final double SENSITIVE_FORWARD_MULTIPLIER = 1;

    public static final double ROTATION_MULTIPLIER = 0.5;
    public static final double SENSITIVE_ROTATION_MULTIPLIER = 0.7;
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

    public static final int CURRENT_LIMIT_AMP = 30;

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
    public static final int CURRENT_LIMIT_AMP = 30;
  }
  
  public static final class Arm {
    public static final int MOTOR_ID = 9;
    public static final int ENCODER_ID = 8;
    public static final int LIMIT_SWITCH_ID = 9;

    public static final int CURRENT_LIMIT_AMP = 20;

    public static final double SPEED_LIMIT = 0.2;
    public static final double CONTROLLER_MULTIPLIER = 0.2;


    // PIDs
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

    // angles

    public static final double ANGLE_FEEDER = 0;
    public static final double ANGLE_THIRD = 100;
    public static final double ANGLE_SECOND = 85;
    public static final double ANGLE_FIRST = 40;
    public static final double ANGLE_REST = 0;
  }

  public static final class Autos {
    public static final class DriveToDistance {
      public static final double KP = 0.7;
      public static final double KI = 0;
      public static final double KD = 0.1;

      public static final double POSITION_TOLERANCE = 0.05;
      public static final double VELOCITY_TOLERANCE = 0.02;
    }

    public static final class TurnByAngle {
      public static final double KP = 2.6;
      public static final double KI = 0;
      public static final double KD = 0.5;

      public static final double POSITION_TOLERANCE = 0.05;
      public static final double VELOCITY_TOLERANCE = 0.02;

    }

    public static final class ReleaseCone {
      public static final double RELEASE_SPEED = 0.8;
      public static final double RELEASE_TIME_SECONDS = 1;
    }

    public static final class ReleaseCube {
      public static final double RELEASE_SPEED = 0.6;
      public static final double RELEASE_TIME_SECONDS = 1;
      public static final double ARM_MOVE_TO_SECOND_TIME_SECONDS = 2;
      public static final double ARM_MOVE_TO_REST_TIME_SECONDS = 2;
    }

    public static final class DriveBackwardsOutsideCommunity {
      public static final double DISTANCE_METERS = 3.5;
    }

    public static final class GetOnChargeStationAuto {
      public static final double DRIVE_SPEED = 0.6;
      public static final double FINISH_ANGLE = 17;
    }

    public static final class ChargeStationBalance {
      public static final boolean IS_REVERSED = true;
      public static final double TIMEOUT_SECONDS_BEFORE_TURNING = 12;
      public static final double TURNING_ANGLE = 90;

      public static final class BangBang {
        public static final double DRIVE_SPEED_FORWARD = 0.23;
        public static final double DRIVE_SPEED_BACKWARDS = 0.35;
        public static final double FINISH_ANGLE = 2;
        public static final double MISTAKE_ANGLE = 8;
        public static final double FINISH_VELOCITY = 4;
        public static final double MISTAKE_ANGLE_BACKWARD = 5;
        public static final double DISTANCE_TO_CENTER = 1.7;
      }

      public static final class PID {
        public static final double KP = 0.017;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double TARGET_ANGLE = 0;
        public static final double POSITION_TOLERANCE = 5;
        public static final double VELOCITY_TOLERANCE = 0.2;
      }

      public static final class Distance {
        public static final double DISTANCE_METERS = 1;
        public static final double WAIT_TIME_SECONDS = 0.5;
      }

    }
  }
}
