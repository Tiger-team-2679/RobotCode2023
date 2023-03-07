package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class MoveArmToPosition extends CommandBase {
  private Arm arm;
  private double targetPosition;
  private double feedForwardResult;
  private final PIDController pid = new PIDController(ArmConstants.Feedforward.KP, ArmConstants.Feedforward.KI, ArmConstants.Feedforward.KD);
  private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.Feedforward.KS, ArmConstants.Feedforward.KG, ArmConstants.Feedforward.KV, ArmConstants.Feedforward.KA);

  public MoveArmToPosition(double targetPosition,Arm arm) {
    this.targetPosition = targetPosition;
    this.arm = arm;
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    pid.setTolerance(ArmConstants.Feedforward.TOLERANCE_POSITION, ArmConstants.Feedforward.TOLERANCE_VELOCITY);
  }

  @Override
  public void execute() {
    double currentPostion = arm.getAngle();
    feedForwardResult = feedforward.calculate(Math.toRadians(targetPosition), 2);
    arm.setSpeed(pid.calculate(currentPostion, targetPosition) + feedForwardResult);
  }

  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
