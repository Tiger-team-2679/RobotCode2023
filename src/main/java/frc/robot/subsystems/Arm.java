package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private static Arm instance = null;
    private final CANSparkMax motor = new CANSparkMax(Constants.Arm.MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_ID);
    private final DigitalInput armLimitSwitch = new DigitalInput(Constants.Arm.LIMIT_SWITCH_ID);
    private final double SPEED_LIMIT = Constants.Arm.SPEED_LIMIT;


    private Arm() {
        motor.setSmartCurrentLimit(Constants.Arm.CURRENT_LIMIT_AMP);
        encoder.setDistancePerRotation(360);
        motor.setInverted(true);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm angle", getAngle());
        SmartDashboard.putBoolean("limit switch", !armLimitSwitch.get());
        if(!armLimitSwitch.get()) resetEncoder();
    }


    public void setSpeed(double speedDemand) {
        motor.set(MathUtil.clamp(speedDemand, -SPEED_LIMIT, SPEED_LIMIT));
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
