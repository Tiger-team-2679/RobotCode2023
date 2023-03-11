package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance = null;
    private final CANSparkMax motorShoulder = new CANSparkMax(ArmConstants.MOTOR_SHOULDER_ID, MotorType.kBrushless);
    private final CANSparkMax motorShoulderFollower = new CANSparkMax(ArmConstants.MOTOR_SHOULDER_FOLLOWER_ID,
            MotorType.kBrushless);
    private final CANSparkMax motorElbow = new CANSparkMax(ArmConstants.MOTOR_ELBOW_ID, MotorType.kBrushless);

    private final DutyCycleEncoder encoderShoulder = new DutyCycleEncoder(ArmConstants.ENCODER_SHOULDER_ID);
    private final DutyCycleEncoder encoderElbow = new DutyCycleEncoder(ArmConstants.ENCODER_ELBOW_ID);
    private final DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_ID);
    private final double SPEED_LIMIT_SHOULDER = ArmConstants.SPEED_LIMIT_SHOULDER;
    private final double SPEED_LIMIT_ELBOW = ArmConstants.SPEED_LIMIT_ELBOW;

    private final ArmFeedforward feedForwardShoulder = new ArmFeedforward(
            ArmConstants.Feedforward.Shoulder.KS,
            ArmConstants.Feedforward.Shoulder.KG,
            ArmConstants.Feedforward.Shoulder.KV,
            ArmConstants.Feedforward.Shoulder.KA);

    private final ArmFeedforward feedforwardElbow = new ArmFeedforward(
            ArmConstants.Feedforward.Elbow.KS,
            ArmConstants.Feedforward.Elbow.KG,
            ArmConstants.Feedforward.Elbow.KV,
            ArmConstants.Feedforward.Elbow.KA);

    private final PIDController pidControllerShoulder = new PIDController(
            ArmConstants.Feedforward.Shoulder.KP,
            ArmConstants.Feedforward.Shoulder.KI,
            ArmConstants.Feedforward.Shoulder.KD);

    private final PIDController pidControllerElbow = new PIDController(
            ArmConstants.Feedforward.Elbow.KP,
            ArmConstants.Feedforward.Elbow.KI,
            ArmConstants.Feedforward.Elbow.KD);

    private boolean isEmergencyMode = false;

    private Arm() {
        motorShoulder.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_SHOULDER_AMP);
        motorShoulderFollower.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_SHOULDER_AMP);
        motorShoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorShoulderFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorShoulderFollower.follow(motorShoulder, true);

        motorElbow.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_ELBOW_AMP);
        motorElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        encoderShoulder.setPositionOffset(ArmConstants.ENCODER_OFFSET_SHOULDER);
        encoderShoulder.setDistancePerRotation(360);
        encoderElbow.setPositionOffset(ArmConstants.ENCODER_OFFSET_ELBOW);
        encoderElbow.setDistancePerRotation(360);

        pidControllerShoulder.setTolerance(
                ArmConstants.Feedforward.Shoulder.TOLERANCE_POSITION,
                ArmConstants.Feedforward.Shoulder.TOLERANCE_VELOCITY);

        pidControllerElbow.setTolerance(
                ArmConstants.Feedforward.Elbow.TOLERANCE_POSITION,
                ArmConstants.Feedforward.Elbow.TOLERANCE_VELOCITY);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm angle shoulder", getShoulderAngle());
        SmartDashboard.putNumber("arm angle shoulder", getElbowAngle());
        SmartDashboard.putBoolean("arm limit switch", !limitSwitch.get());
        SmartDashboard.putBoolean("arm is safety mode", isEmergencyMode);
    }

    public void setSpeedShoulder(double demand) {
        motorShoulder.set(isEmergencyMode && !limitSwitch.get() && demand < 0
                ? 0
                : MathUtil.clamp(demand, -SPEED_LIMIT_SHOULDER, SPEED_LIMIT_SHOULDER));
    }

    public void setSpeedElbow(double demand) {
        motorElbow.set(MathUtil.clamp(demand, -SPEED_LIMIT_ELBOW, SPEED_LIMIT_ELBOW));
    }

    public void setVoltageShoulder(double demand) {
        motorShoulder.setVoltage(isEmergencyMode && !limitSwitch.get() && demand < 0
                ? 0
                : MathUtil.clamp(demand, -12, 12));
    }

    public void setVoltageElbow(double demand) {
        motorElbow.setVoltage(MathUtil.clamp(demand, -12, 12));
    }

    public double getShoulderAngle() {
        return -encoderShoulder.getDistance();
    }

    public double getElbowAngle() {
        return -encoderShoulder.getDistance();
    }

    public ArmValues<Double> calculateFeedforward(
            double shoulderAngle,
            double elbowAngle,
            double shoulderVelocity,
            double elbowVelocity,
            boolean usePID) {

        if (isEmergencyMode)
            return new ArmValues<Double>(0.0, 0.0);

        ArmValues<Double> voltages = new ArmValues<>(
                feedForwardShoulder.calculate(Math.toRadians(shoulderAngle), shoulderVelocity),
                feedforwardElbow.calculate(Math.toRadians(elbowAngle), elbowVelocity));

        if (usePID) {
            voltages.shoulder += pidControllerShoulder.calculate(getShoulderAngle(), shoulderAngle);
            voltages.elbow += pidControllerElbow.calculate(getElbowAngle(), elbowAngle);
        }

        return voltages;
    }

    public void resetPIDs() {
        pidControllerShoulder.reset();
        pidControllerElbow.reset();
    }

    public boolean pidsAtSetpoints() {
        return pidControllerShoulder.atSetpoint() && pidControllerElbow.atSetpoint();
    }

    public void setEmergencyMode(boolean isLimitSwitchSafetyMode) {
        this.isEmergencyMode = isLimitSwitchSafetyMode;
    }

    public boolean getEmergencyMode() {
        return isEmergencyMode;
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }
}
