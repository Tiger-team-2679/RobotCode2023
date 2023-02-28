package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance = null;
    private final TalonSRX leftMotor = new TalonSRX(Constants.Drivetrain.LEFT_ID);
    private final TalonSRX leftMotorFollower = new TalonSRX(Constants.Drivetrain.LEFT_FOLLOWER_ID);
    private final TalonSRX rightMotor = new TalonSRX(Constants.Drivetrain.RIGHT_ID);
    private final TalonSRX rightMotorFollower = new TalonSRX(Constants.Drivetrain.RIGHT_FOLLOWER_ID);

    private final PigeonIMU imu = new PigeonIMU(rightMotorFollower);
    private final Encoder leftEncoder = new Encoder(Constants.Drivetrain.LEFT_ENCODER_CHANNEL_A,
            Constants.Drivetrain.LEFT_ENCODER_CHANNEL_B);
    private final Encoder rightEncoder = new Encoder(Constants.Drivetrain.RIGHT_ENCODER_CHANNEL_A,
            Constants.Drivetrain.RIGHT_ENCODER_CHANNEL_B);

    private final PIDController leftVelocityPID = new PIDController(Constants.Drivetrain.VELOCITY_KP, Constants.Drivetrain.VELOCITY_KI,
                    Constants.Drivetrain.VELOCITY_KD);

    private final PIDController rightVelocityPID = new PIDController(Constants.Drivetrain.VELOCITY_KP, Constants.Drivetrain.VELOCITY_KI,
            Constants.Drivetrain.VELOCITY_KD);

    private final PIDController leftVoltagePID = new PIDController(Constants.Drivetrain.VOLTAGE_KP, Constants.Drivetrain.VOLTAGE_KI,
                    Constants.Drivetrain.VOLTAGE_KD);

    private final PIDController rightVoltagePID = new PIDController(Constants.Drivetrain.VOLTAGE_KP, Constants.Drivetrain.VOLTAGE_KI,
                    Constants.Drivetrain.VOLTAGE_KD);

    private double lastSpeedLeft = 0;
    private double lastSpeedRight = 0;
    private ControlType controlType = ControlType.VOLTAGE;

    enum ControlType{
        VOLTAGE,
        VELOCITY,
        GRADUAL_VOLTAGE
    }

    private Drivetrain() {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
                true,
                90,
                0,
                0
        );
        rightMotor.configSupplyCurrentLimit(currentLimitConfiguration);
        rightMotorFollower.configSupplyCurrentLimit(currentLimitConfiguration);
        leftMotor.configSupplyCurrentLimit(currentLimitConfiguration);
        leftMotorFollower.configSupplyCurrentLimit(currentLimitConfiguration);

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
        double roundsPerPules = 1.0 / pulsesInRound;

        double distancePerPules = distancePerRound * roundsPerPules;    

        rightEncoder.setDistancePerPulse(distancePerPules);
        leftEncoder.setDistancePerPulse(distancePerPules);
    }


    private void set(double leftDemand, double rightDemand) {
        leftDemand = MathUtil.clamp(leftDemand, -1, 1);
        rightDemand = MathUtil.clamp(rightDemand, -1, 1);
        leftMotor.set(ControlMode.PercentOutput, leftDemand);
        rightMotor.set(ControlMode.PercentOutput, rightDemand);
        lastSpeedLeft = leftDemand;
        lastSpeedRight = rightDemand;
    }

    public void setVelocity(double leftDemand, double rightDemand) {
        controlType = ControlType.VELOCITY;
        leftVelocityPID.setSetpoint(leftDemand);
        rightVelocityPID.setSetpoint(rightDemand);
    }

    public void setGradualSpeed(double leftDemand, double rightDemand){
        controlType = ControlType.GRADUAL_VOLTAGE;
        leftVoltagePID.setSetpoint(leftDemand);
        rightVoltagePID.setSetpoint(rightDemand);
    }

    public void setSpeed(double leftDemand, double rightDemand){
        controlType = ControlType.VOLTAGE;
        set(leftDemand, rightDemand);
    }

    public double getPitch() {
        return imu.getPitch();
    }

    public double getYaw() {
        return imu.getYaw();
    }

    public void setYaw(double angle) {
        imu.setYaw(angle);
    }

    public double getLeftDistanceMeters() {
        return leftEncoder.getDistance();
    }

    public double getRightDistanceMeters() {
        return rightEncoder.getDistance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IMU pitch", getPitch());

        if(controlType == ControlType.VELOCITY || controlType == ControlType.GRADUAL_VOLTAGE){
            double leftPIDValue = controlType == ControlType.VELOCITY 
                ? leftVelocityPID.calculate(leftEncoder.getRate() / Constants.Drivetrain.MAX_VELOCITY)
                : leftVoltagePID.calculate(lastSpeedLeft);
            double rightPIDValue = controlType == ControlType.VELOCITY 
                ? rightVelocityPID.calculate(rightEncoder.getRate() / Constants.Drivetrain.MAX_VELOCITY)
                : rightVoltagePID.calculate(lastSpeedRight);
                
            double finalLeftValue = MathUtil.clamp(lastSpeedLeft + leftPIDValue, -1, 1);
            double finalRightValue = MathUtil.clamp(lastSpeedRight + rightPIDValue, -1, 1);

            set(finalLeftValue, finalRightValue);
        }
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }

        return instance;
    }
}
