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

  private final ArmFeedforward feedForwardShoulder = new ArmFeedforward(
      ArmConstants.Feedforward.Shoulder.KS,
      ArmConstants.Feedforward.Shoulder.KG,
      ArmConstants.Feedforward.Shoulder.KV,
      ArmConstants.Feedforward.Shoulder.KA);

  private final ArmFeedforward feedforwardElbow = new ArmFeedforward(
      ArmConstants.Feedforward.Elbow.KS,
      ArmConstants.Feedforward.Elbow.KG,
      ArmConstants.Feedforward.Elbow.KV,
      ArmConstants.Feedforward.Elbow.KA);

  private final PIDController pidControllerShoulder = new PIDController(
      ArmConstants.Feedforward.Shoulder.KP,
      ArmConstants.Feedforward.Shoulder.KI,
      ArmConstants.Feedforward.Shoulder.KD);

  private final PIDController pidControllerElbow = new PIDController(
      ArmConstants.Feedforward.Elbow.KP,
      ArmConstants.Feedforward.Elbow.KI,
      ArmConstants.Feedforward.Elbow.KD);

  private TrapezoidProfile trapezoidProfileShoulder;
  private TrapezoidProfile trapezoidProfileElbow;

  private final double targetPositionShoulder;
  private final double targetPositionElbow;

  private final Timer timer = new Timer();

  public MoveArmToPosition(Arm arm, double targetPositionShoulder, double targetPositionElbow) {
    this.arm = arm;
    addRequirements(this.arm);

    this.targetPositionShoulder = targetPositionShoulder;
    this.targetPositionElbow = targetPositionElbow;
  }

  @Override
  public void initialize() {
    timer.restart();

    pidControllerShoulder.setTolerance(
        ArmConstants.Feedforward.Shoulder.TOLERANCE_POSITION,
        ArmConstants.Feedforward.Shoulder.TOLERANCE_VELOCITY);
    pidControllerElbow.setTolerance(
        ArmConstants.Feedforward.Elbow.TOLERANCE_POSITION,
        ArmConstants.Feedforward.Elbow.TOLERANCE_VELOCITY);

    trapezoidProfileShoulder = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ArmConstants.Feedforward.Shoulder.MAX_VELOCITY,
            ArmConstants.Feedforward.Shoulder.MAX_ACCELERATION),
        new TrapezoidProfile.State(targetPositionShoulder, 0),
        new TrapezoidProfile.State(arm.getShoulderAngle(), 0));

    trapezoidProfileElbow = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ArmConstants.Feedforward.Elbow.MAX_VELOCITY,
            ArmConstants.Feedforward.Elbow.MAX_ACCELERATION),
        new TrapezoidProfile.State(targetPositionElbow, 0),
        new TrapezoidProfile.State(arm.getElbowAngle(), 0));
  }

  @Override
  public void execute() {
    TrapezoidProfile.State setpointsShoulder = trapezoidProfileShoulder.calculate(timer.get());
    TrapezoidProfile.State setpointsElbow = trapezoidProfileElbow.calculate(timer.get());

    double feedForwardResultShoulder = feedForwardShoulder.calculate(
        Math.toRadians(setpointsShoulder.position),
        setpointsShoulder.velocity);
    double feedforwardResultElbow = feedforwardElbow.calculate(
        Math.toRadians(setpointsElbow.position),
        setpointsElbow.velocity);

    double demandVoltageShoulder = pidControllerShoulder.calculate(arm.getShoulderAngle(), setpointsShoulder.position)
        + feedForwardResultShoulder;
    double demandVoltageElbow = pidControllerElbow.calculate(arm.getShoulderAngle(), setpointsElbow.position)
        + feedforwardResultElbow;

    arm.setVoltageShoulder(demandVoltageShoulder);
    arm.setVoltageElbow(demandVoltageElbow);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return pidControllerShoulder.atSetpoint() && pidControllerElbow.atSetpoint();
  }
}
