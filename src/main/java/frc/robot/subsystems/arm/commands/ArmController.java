package frc.robot.subsystems.arm.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmValues;

public class ArmController extends CommandBase {
  private final Arm arm;

  private final DoubleSupplier shoulderDemandSupplier;
  private final DoubleSupplier elbowDemandSupplier;

  public ArmController(Arm arm, DoubleSupplier shoulderDemandSupplier, DoubleSupplier elbowDemandSupplier) {
    this.arm = arm;
    addRequirements(arm);
    this.shoulderDemandSupplier = shoulderDemandSupplier;
    this.elbowDemandSupplier = elbowDemandSupplier;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double shoulderDemand = shoulderDemandSupplier.getAsDouble();
    double elbowDemand = elbowDemandSupplier.getAsDouble();

    shoulderDemand = MathUtil.clamp(shoulderDemand, -1, 1);
    shoulderDemand = MathUtil.applyDeadband(shoulderDemand, Constants.OI.JOYSTICKS_DEADBAND_VALUE);
    elbowDemand = MathUtil.clamp(elbowDemand, -1.0, 1.0);
    elbowDemand = MathUtil.applyDeadband(elbowDemand, Constants.OI.JOYSTICKS_DEADBAND_VALUE);

    shoulderDemand *= ArmConstants.Controller.MULTIPLIER_SHOULDER * 12;
    elbowDemand *= ArmConstants.Controller.MULTIPLIER_ELBOW * 12;

    ArmValues<Double> feedforwardResults = arm.calculateFeedforward(
      arm.getShoulderAngle(),
      arm.getElbowAngle(),
      0,
      0,
      false
    );

    arm.setVoltageShoulder(shoulderDemand + feedforwardResults.shoulder);
    arm.setVoltageElbow(elbowDemand + feedforwardResults.elbow);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
