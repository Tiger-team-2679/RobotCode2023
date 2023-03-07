package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class MoveArmToPosition extends CommandBase {
  private final Arm arm;
  private double feedForwardResult;
  private final PIDController pid = new PIDController(ArmConstants.Feedforward.KP, ArmConstants.Feedforward.KI,
      ArmConstants.Feedforward.KD);
  private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.Feedforward.KS,
      ArmConstants.Feedforward.KG, ArmConstants.Feedforward.KV, ArmConstants.Feedforward.KA);
  private final TrapezoidProfile trapezoidProfile;
  private final Timer timer = new Timer();

  public MoveArmToPosition(double targetPosition, Arm arm) {
    this.arm = arm;
    addRequirements(this.arm);

    trapezoidProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(ArmConstants.Feedforward.MAX_VELOCITY, ArmConstants.Feedforward.MAX_ACCELERATION),
      new TrapezoidProfile.State(targetPosition, 0),
      new TrapezoidProfile.State(arm.getAngle(), 0)
    );
  }

  @Override
  public void initialize() {
    pid.setTolerance(ArmConstants.Feedforward.TOLERANCE_POSITION, ArmConstants.Feedforward.TOLERANCE_VELOCITY);
    timer.restart();
  }

  @Override
  public void execute() {
    double currentPostion = arm.getAngle();
    TrapezoidProfile.State setpoints = trapezoidProfile.calculate(timer.get());
    feedForwardResult = feedforward.calculate(Math.toRadians(setpoints.position), setpoints.velocity);
    arm.setSpeed(pid.calculate(currentPostion, setpoints.position) + feedForwardResult);
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
