package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake instance = null;
    private final TalonSRX motor = new TalonSRX(Constants.Intake.MOTOR_ID);

    private Intake() {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
                true,
                Constants.Intake.CURRENT_LIMIT_AMP,
                0,
                0
        );
        motor.configSupplyCurrentLimit(currentLimitConfiguration);
    }

    public void setSpeed(double demand) {
        demand = MathUtil.clamp(demand, -1, 1);
        motor.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void periodic() { }

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }
}
