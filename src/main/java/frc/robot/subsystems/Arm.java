package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.Arm.MOTOR_ID, MotorType.kBrushless);
    private static Arm instance = null;
    private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_ID);
    

    private Arm() {
        // setDefaultCommand(new ArmJoystick(this));
        encoder.setDistancePerRotation(360);
        motor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("get()", getAngle());
        SmartDashboard.putNumber("getPositionOffset()", encoder.getPositionOffset());
        // This method will be called once per scheduler run
    }


    public void setSpeed(double speedDemand) {
        motor.set(speedDemand);
    }

    public double getAngle() {
        return encoder.getDistance();
    }


    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }
}
