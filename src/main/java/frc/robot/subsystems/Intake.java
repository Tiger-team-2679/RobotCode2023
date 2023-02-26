package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonSRX motor = new TalonSRX(Constants.Intake.MOTOR_ID);
    private static Intake instance = null;

    private Intake() {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 90, 0, 0);
        motor.configSupplyCurrentLimit(currentLimitConfiguration);
    }

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void setSpeed(double demand) {
        motor.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void periodic() {
    }
}
