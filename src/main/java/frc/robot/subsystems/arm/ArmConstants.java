package frc.robot.subsystems.arm;

public final class ArmConstants {
  public static final int MOTOR_SHOULDER_ID = 9;
  public static final int MOTOR_SHOULDER_FOLLOWER_ID = 10;
  public static final int MOTOR_ELBOW_ID = 11;
  public static final int ENCODER_SHOULDER_ID = 6;
  public static final int ENCODER_ELBOW_ID = 7;
  public static final int LIMIT_SWITCH_ID = 10;
  public static final double ENCODER_OFFSET_SHOULDER = 0.3011;
  public static final double ENCODER_OFFSET_ELBOW = 0.3011;

  public static final int CURRENT_LIMIT_SHOULDER_AMP = 20;
  public static final int CURRENT_LIMIT_ELBOW_AMP = 20;

  public static final double SPEED_LIMIT_SHOULDER = 0.2;
  public static final double SPEED_LIMIT_ELBOW = 0.2;

  public static final double ANGLE_FEEDER_SHOULDER = 10, ANGLE_FEEDER_ELBOW = 0;
  public static final double ANGLE_THIRD_SHOULDER = 0, ANGLE_THIRD_ELBOW = 0;
  public static final double ANGLE_SECOND_SHOULDER = -30, ANGLE_SECOND_ELBOW = 0;
  public static final double ANGLE_FIRST_SHOULDER = -60, ANGLE_FIRST_ELBOW = 0;
  public static final double ANGLE_REST_SHOULDER = 0, ANGLE_REST_ELBOW = 0;

  public static final class Feedforward {
    public static final class Shoulder {
      public static final double KP = 0.0;
      public static final double KI = 0.00;
      public static final double KD = 0.0;

      public static final double KV = 0.04;
      public static final double KG = 0.48;
      public static final double KS = 0;
      public static final double KA = 0.0;

      public static final double TOLERANCE_POSITION = 0;
      public static final double TOLERANCE_VELOCITY = 0;
      public static final double MAX_VELOCITY = 100;
      public static final double MAX_ACCELERATION = 150;
    }

    public static final class Elbow {
      public static final double KP = 0.0;
      public static final double KI = 0.00;
      public static final double KD = 0.0;

      public static final double KV = 0.04;
      public static final double KG = 0.48;
      public static final double KS = 0;
      public static final double KA = 0.0;

      public static final double TOLERANCE_POSITION = 0;
      public static final double TOLERANCE_VELOCITY = 0;
      public static final double MAX_VELOCITY = 100;
      public static final double MAX_ACCELERATION = 150;
    }
  }

  public static final class PID {
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

    public static final double KP_MAX = 2.5;
    public static final double KI_MAX = 0.019;
    public static final double KD_MAX = 0.05;

    public static final double KP_FEEDER = 0;
    public static final double KI_FEEDER = 0;
    public static final double KD_FEEDER = 0;
  }

  public static final class Controller {
    public static final double MULTIPLIER_SHOULDER = 0.2;
    public static final double MULTIPLIER_ELBOW = 0.2;
  }

  public static final class DoubleJointArmFeedforward {
    public static final double NOMINAL_VOLTAGE = 12.0; // [V]
    public static final double STALL_CURRENT = 105; // [A]
    public static final double STALL_TORQUE = 2.6; // [N*m]
    public static final double FREE_SPEED = (5676 / 60.0) * (2 * Math.PI); // [rad/s]
    public static final double Kt = STALL_TORQUE / STALL_CURRENT; // [N*m/A]
    public static final double Kv = FREE_SPEED / NOMINAL_VOLTAGE; // [rad/s/V]
    public static final double R = NOMINAL_VOLTAGE / STALL_CURRENT; // [Ohm]
  }

  public static final class Joints {
    public static final class Shoulder {
      public static final double MASS = 0;
      public static final double LENGTH = 0;
      public static final double CM_RADIUS = 0;
      public static final double MOMENT_OF_INERTIA = 0;
      public static final double GEARING = 0;
      public static final int NUMBER_OF_MOTORS = 0;
    }

    public static final class Elbow {
      public static final double MASS = 0;
      public static final double LENGTH = 0;
      public static final double CM_RADIUS = 0;
      public static final double MOMENT_OF_INERTIA = 0;
      public static final double GEARING = 0;
      public static final int NUMBER_OF_MOTORS = 0;
    }
  }
}
