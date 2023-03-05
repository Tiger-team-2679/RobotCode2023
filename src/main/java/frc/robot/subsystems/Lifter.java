package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lifter extends SubsystemBase {
  private static Lifter instance = null;
  private final TalonSRX motor = new TalonSRX(Constants.Lifter.MOTOR_ID);

  private Lifter() {
      SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
              true,
              Constants.Lifter.CURRENT_LIMIT_AMP,
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

  public static Lifter getInstance() {
      if(instance == null) {
          instance = new Lifter();
      }
      return instance;
  }
}
