package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class HoldArmPosition extends CommandBase {
  private Arm arm;
  private double targetPosition;
  private double feedForwardResult;
  private ArmFeedforward feedforward = new ArmFeedforward(
      ArmConstants.Feedforward.KS,
      ArmConstants.Feedforward.KG,
      ArmConstants.Feedforward.KV,
      ArmConstants.Feedforward.KA);

  public HoldArmPosition(Arm arm) {
    this.arm = arm;
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    targetPosition = arm.getAngle();
    feedForwardResult = feedforward.calculate(Math.toRadians(targetPosition), 0);
  }

  @Override
  public void execute() {
    arm.setVoltage(feedForwardResult);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
