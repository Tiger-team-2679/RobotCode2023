package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArmJoystick;

public class Arm extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.Arm.MOTOR_ID, MotorType.kBrushless);
    private static Arm instance = null;
    private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_ID);
    

    private Arm() {
        setDefaultCommand(new ArmJoystick(this));
        encoder.setDistancePerRotation(360);
        motor.setInverted(true);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("get() angle encoder arm", getAngle());
        // This method will be called once per scheduler run
    }


    public void setSpeed(double speedDemand) {
        motor.set(speedDemand);
    }

    public double getTorque(double angle){
        double r = 0.2; //radius m
        double f = 0.1;//weight kg
        return f * r * Math.sin(angle);
    }

    public double getTorqueToSpeed(double t){
        double x = 2222.222*t - 5888.88;
        double power = -0.00000027*x*x + 0.00162*x + 0.003;
        //power = speed * torque
        return (power / t);
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
    public void resetEncoder2(){
        encoder.setPositionOffset(50);
    }
}
