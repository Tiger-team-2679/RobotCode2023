package frc.robot.subsystems.drivetrain;

public final class DrivetrainConstants {
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

    public static final class ArcadeDrive {
        public static final double FORWARD_MULTIPLIER = 0.7;
        public static final double SENSITIVE_FORWARD_MULTIPLIER = 1;

        public static final double ROTATION_MULTIPLIER = 0.5;
        public static final double SENSITIVE_ROTATION_MULTIPLIER = 0.7;
    }
}
