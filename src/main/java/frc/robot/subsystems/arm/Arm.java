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
    private final CANSparkMax motor = new CANSparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);
    private final DigitalInput armLimitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_ID);
    private final double SPEED_LIMIT = ArmConstants.SPEED_LIMIT;


    private Arm() {
        motor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_AMP);
        encoder.setDistancePerRotation(360);
        motor.setInverted(true);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        encoder.setPositionOffset(ArmConstants.ENCODER_OFFSET);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm angle", getAngle());
        SmartDashboard.putBoolean("limit switch", !armLimitSwitch.get());
        SmartDashboard.putNumber("arm angle positionOffset", encoder.getPositionOffset());
    
        if(!armLimitSwitch.get()) resetEncoder();
    }


    public void setSpeed(double speedDemand) {
        motor.set(MathUtil.clamp(speedDemand, -SPEED_LIMIT, SPEED_LIMIT));
    }
    public void setVoltage(double speedDemand){
        motor.setVoltage(MathUtil.clamp(speedDemand, -12, 12));
    }

    public double getAngle() {
        return -encoder.getDistance();
    }

    public void resetEncoder(){
        encoder.reset();
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }
}
