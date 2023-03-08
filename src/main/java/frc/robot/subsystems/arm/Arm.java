package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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

    private boolean isLimitSwitchSafetyMode = true;

    private Arm() {
        motorShoulder.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_ELBOW_AMP);
        motorShoulderFollower.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_ELBOW_AMP);
        motorShoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorShoulderFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorShoulderFollower.follow(motorShoulder, true);

        motorElbow.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_SHOULDER_AMP);
        motorElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        encoderShoulder.setPositionOffset(ArmConstants.ENCODER_OFFSET_SHOULDER);
        encoderShoulder.setDistancePerRotation(360);
        encoderElbow.setPositionOffset(ArmConstants.ENCODER_OFFSET_ELBOW);
        encoderElbow.setDistancePerRotation(360);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm angle shoulder", getShoulderAngle());
        SmartDashboard.putNumber("arm angle shoulder", getElbowAngle());
        SmartDashboard.putBoolean("arm limit switch", !limitSwitch.get());
        SmartDashboard.putBoolean("arm is safety mode", isLimitSwitchSafetyMode);
    }

    public void setSpeedShoulder(double demand) {
        motorShoulder.set(isLimitSwitchSafetyMode && !limitSwitch.get() && demand < 0
                ? 0
                : MathUtil.clamp(demand, -SPEED_LIMIT_SHOULDER, SPEED_LIMIT_SHOULDER));
    }

    public void setSpeedElbow(double demand) {
        motorElbow.set(MathUtil.clamp(demand, -SPEED_LIMIT_ELBOW, SPEED_LIMIT_ELBOW));
    }

    public void setVoltageShoulder(double demand) {
        motorShoulder.setVoltage(isLimitSwitchSafetyMode && !limitSwitch.get() && demand < 0
                ? 0
                : MathUtil.clamp(demand, -12, 12));
    }

    public void setVoltageElbow(double demand) {
        motorShoulder.setVoltage(MathUtil.clamp(demand, -12, 12));
    }

    public double getShoulderAngle() {
        return -encoderShoulder.getDistance();
    }

    public double getElbowAngle() {
        return -encoderShoulder.getDistance();
    }

    public void setLimitSwitchSafetyMode(boolean isLimitSwitchSafetyMode) {
        this.isLimitSwitchSafetyMode = isLimitSwitchSafetyMode;
    }

    public boolean getLimitSwitchSafetyMode() {
        return isLimitSwitchSafetyMode;
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }
}
