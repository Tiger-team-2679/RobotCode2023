package frc.robot.subsystems.arm;

public final class ArmConstants {
    public static final int MOTOR_ID = 9;
    public static final int ENCODER_ID = 8;
    public static final int LIMIT_SWITCH_ID = 9;
    public static final double ENCODER_OFFSET = 0.5456;

    public static final int CURRENT_LIMIT_AMP = 30;

    public static final double SPEED_LIMIT = 0.2;

    public static final double ANGLE_FEEDER = 0;
    public static final double ANGLE_THIRD = 100;
    public static final double ANGLE_SECOND = 85;
    public static final double ANGLE_FIRST = 40;
    public static final double ANGLE_REST = 0;

    public static final class Feedforward {
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double KV = 0.04;
        public static final double KG = 0.35;
        public static final double KS = 0;
        public static final double KA = 0;

        public static final double TOLERANCE_POSITION = 0;
        public static final double TOLERANCE_VELOCITY = 0;
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
        public static final double MULTIPLIER = 0.2;
    }
}
