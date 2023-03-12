package frc.robot;

public final class Constants {
  public static final class OI {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double JOYSTICKS_DEADBAND_VALUE = 0.1;
  }

  public static final class Autos {
    public static final class DriveToDistance {
      public static final double KP = 0.7;
      public static final double KI = 0;
      public static final double KD = 0.1;

      public static final double POSITION_TOLERANCE = 0.10;
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
      public static final double ARM_MOVE_TO_SECOND_TIME_SECONDS = 2.5;
      public static final double ARM_MOVE_TO_REST_TIME_SECONDS = 2;
    }

    public static final class ReleaseCubeToThird {
      public static final double RELEASE_SPEED = 0.8;
      public static final double RELEASE_TIME_SECONDS = 1;
      public static final double ARM_MOVE_TO_SECOND_TIME_SECONDS = 2.5;
      public static final double ARM_MOVE_TO_REST_TIME_SECONDS = 2;
    }

    public static final class DriveBackwardsOutsideCommunity {
      public static final double DISTANCE_METERS = 3.5;
      public static final double TURN_ENGLE = 180;
      public static final double WAIT_TIME = 3.5;
    }

    public static final class GetOnChargeStationAuto {
      public static final double DRIVE_SPEED = 0.6;
      public static final double FINISH_ANGLE = 17;
      public static final double TIMEOUT_SECONDS = 4.5;
    }

    public static final class BalanceOnChargeStationAuto {
      public static final boolean IS_REVERSED = true;

      public static final double KP = 0.017;
      public static final double KI = 0;
      public static final double KD = 0;

      public static final double TARGET_ANGLE = 0;
      public static final double POSITION_TOLERANCE = 1.5;
      public static final double VELOCITY_TOLERANCE = 0.2;
    }
  }
}
