package frc.robot.subsystems.arm.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class ArmController extends CommandBase {
  private final Arm arm;

  private final DoubleSupplier shoulderDemandSupplier;
  private final DoubleSupplier elbowDemandSupplier;

  private ArmFeedforward shoulderFeedforward = new ArmFeedforward(
      ArmConstants.Feedforward.Shoulder.KS,
      ArmConstants.Feedforward.Shoulder.KG,
      ArmConstants.Feedforward.Shoulder.KV,
      ArmConstants.Feedforward.Shoulder.KA);

  private ArmFeedforward elbowFeedforward = new ArmFeedforward(
      ArmConstants.Feedforward.Elbow.KS,
      ArmConstants.Feedforward.Elbow.KG,
      ArmConstants.Feedforward.Elbow.KV,
      ArmConstants.Feedforward.Elbow.KA);

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

    shoulderDemand *= ArmConstants.Controller.MULTIPLIER_SHOULDER;
    elbowDemand *= ArmConstants.Controller.MULTIPLIER_ELBOW;

    if (shoulderDemand == 0) {
      double targetPosition = arm.getShoulderAngle();
      arm.setVoltageShoulder(shoulderFeedforward.calculate(Math.toRadians(targetPosition), 0));
    } else {
      arm.setSpeedShoulder(shoulderDemand);
    }

    if (elbowDemand == 0) {
      double targetPosition = arm.getElbowAngle();
      arm.setVoltageElbow(elbowFeedforward.calculate(Math.toRadians(targetPosition), 0));
    } else {
      arm.setSpeedElbow(elbowDemand);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
