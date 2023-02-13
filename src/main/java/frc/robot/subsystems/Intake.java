package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.IntakeController;

public class Intake extends SubsystemBase {
    private TalonSRX motor = new TalonSRX(Constants.Intake.MOTOR_ID);
    private static Intake instance = null;

    private Intake() {
        setDefaultCommand(new IntakeController(this));
    }

    public static Intake getInstance() {
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    public void setSpeed(double demand) {
        motor.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
