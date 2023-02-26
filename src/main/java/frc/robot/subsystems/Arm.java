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
    private CANSparkMax motor = new CANSparkMax(Constants.Arm.MOTOR_ID, MotorType.kBrushless);
    private static Arm instance = null;
    private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_ID);
    private DigitalInput armlimitSwitch = new DigitalInput(Constants.Arm.LIMITSWITCH_ID);    

    private Arm() {
        motor.setSmartCurrentLimit(20);
        encoder.setDistancePerRotation(360);
        motor.setInverted(true);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        if(motor.getOutputCurrent() > maxCurrentTested) maxCurrentTested = motor.getOutputCurrent();
        SmartDashboard.putNumber("get() angle encoder arm", getAngle());
        SmartDashboard.putBoolean("limit switch", !armlimitSwitch.get());
        if(!armlimitSwitch.get())resetEncoder();
        // This method will be called once per scheduler run
    }


    public void setSpeed(double speedDemand) {
        motor.set(MathUtil.clamp(speedDemand, -0.2, 0.2));
    }

    public double getAngle() {
        return -encoder.getDistance();
    }


    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void resetEncoder(){
        encoder.reset();
    }
}
