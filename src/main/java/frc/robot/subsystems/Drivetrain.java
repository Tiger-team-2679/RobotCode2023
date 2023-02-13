package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;

public class Drivetrain extends SubsystemBase {
    private final TalonSRX leftMotor = new TalonSRX(Constants.Drivetrain.LEFT_ID);
    private final TalonSRX leftMotorFollower = new TalonSRX(Constants.Drivetrain.LEFT_FOLLOWER_ID);
    private final TalonSRX rightMotor = new TalonSRX(Constants.Drivetrain.RIGHT_ID);
    private final TalonSRX rightMotorFollower = new TalonSRX(Constants.Drivetrain.RIGHT_FOLLOWER_ID);

    private final PigeonIMU imu = new PigeonIMU(leftMotorFollower);
    private final Encoder leftEncoder = new Encoder(Constants.Drivetrain.LEFT_ENCODER_CHANNEL_A,
            Constants.Drivetrain.LEFT_ENCODER_CHANNEL_B);
    private final Encoder rightEncoder = new Encoder(Constants.Drivetrain.RIGHT_ENCODER_CHANNEL_A,
            Constants.Drivetrain.RIGHT_ENCODER_CHANNEL_B);

    private final PIDController velocityPID = new PIDController(Constants.ArcadeDrive.KP, Constants.ArcadeDrive.KI,
            Constants.ArcadeDrive.KD);

            
    
    private double targetValocityLeft = 0;
    private double targetValocityRight = 0;
    private double lastSpeedLeft = 0;
    private double lastSpeedright = 0;

    private static Drivetrain instance = null;
    private boolean isUsingVelocity = false;

    /** Creates a new Drivetrain. */
    private Drivetrain() {
        setDefaultCommand(new ArcadeDrive(this));

        leftMotorFollower.follow(leftMotor);
        rightMotorFollower.follow(rightMotor);

        rightMotor.setInverted(true);
        rightMotorFollower.setInverted(true);

        rightMotor.setNeutralMode(NeutralMode.Brake);
        rightMotorFollower.setNeutralMode(NeutralMode.Brake);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        leftMotorFollower.setNeutralMode(NeutralMode.Brake);

        rightEncoder.setReverseDirection(true);

        double wheelRadiusInMeters = 0.076;
        int pulsesInRound = 2048;

        double distancePerRound = wheelRadiusInMeters * 2 * Math.PI;
        double roundsPerPules = 1 / (double) pulsesInRound;

        double distancePerPules = distancePerRound * roundsPerPules;

        rightEncoder.setDistancePerPulse(distancePerPules);
        leftEncoder.setDistancePerPulse(distancePerPules);

        imu.setYaw(0);
    }

    private void set(double leftDemand, double rightDemand) {
        leftMotor.set(ControlMode.PercentOutput, leftDemand);
        rightMotor.set(ControlMode.PercentOutput, rightDemand);
        lastSpeedLeft = leftDemand;
        lastSpeedright = rightDemand;
    }

    public void setVelocity(double leftDemand, double rightDemand) {
        isUsingVelocity = true;
        targetValocityLeft = leftDemand;
        targetValocityRight = rightDemand;
    }

    public void setSpeed(double leftDemand, double rightDemand){
        isUsingVelocity = false;
        set(leftDemand, rightDemand);
    }

    public double getYaw() {
        return imu.getYaw();
    }

    public double getLeftDistanceMeters() {
        return leftEncoder.getDistance();
    }

    public double getRightDistanceMeters() {
        return rightEncoder.getDistance();
    }

    /**
     * * @return speed of the left motors from encoder, in meters per seconds
     */
    public double getLeftSpeed() {
        return leftEncoder.getRate();
    }

    /**
     * * @return speed of the right motors from encoder, in meters per seconds
     */
    public double getRightSpeed() {
        return rightEncoder.getRate();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isUsingVelocity", isUsingVelocity);
        if(isUsingVelocity){
            double leftPIDValue = velocityPID.calculate(getLeftSpeed() / Constants.ArcadeDrive.MAX_SPEED, targetValocityLeft);
            double rightPIDValue = velocityPID.calculate(getRightSpeed() / Constants.ArcadeDrive.MAX_SPEED, targetValocityRight);

            SmartDashboard.putNumber("leftDemand", targetValocityLeft);
            SmartDashboard.putNumber("rightDemand", targetValocityRight);
            double finalLeftValue = MathUtil.clamp(lastSpeedLeft + leftPIDValue, -1, 1);
            double finalRightValue = MathUtil.clamp(lastSpeedright + rightPIDValue, -1, 1);

            set(finalLeftValue, finalRightValue);
        }

        SmartDashboard.putNumber("leftSpeed", getRightSpeed() / Constants.ArcadeDrive.MAX_SPEED);
        SmartDashboard.putNumber("rightSpeed", getLeftSpeed() / Constants.ArcadeDrive.MAX_SPEED);
        SmartDashboard.putNumber("Right Encoder Distance", getRightDistanceMeters());
        SmartDashboard.putNumber("Left Encoder Distance", getLeftDistanceMeters());
        SmartDashboard.putNumber("delta left", MathUtil.clamp(Math.abs((getLeftSpeed() / Constants.ArcadeDrive.MAX_SPEED)-(targetValocityLeft)), 0, 0.05));
        SmartDashboard.putNumber("delta right", MathUtil.clamp(Math.abs((getRightSpeed() / Constants.ArcadeDrive.MAX_SPEED)-(targetValocityRight)), 0, 0.05));
        SmartDashboard.putNumber("PigeonIMU Yaw", getYaw());

    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }

        return instance;
    }
}
