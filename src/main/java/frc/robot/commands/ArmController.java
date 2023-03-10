package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmController extends CommandBase {
  private final Arm arm;
  private final DoubleSupplier demandSupplier;

  public ArmController(Arm arm, DoubleSupplier demandSupplier) {
    this.arm = arm;
    addRequirements(arm);
    this.demandSupplier = demandSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double demand = demandSupplier.getAsDouble();
    demand = MathUtil.clamp(demand, -1.0, 1.0);
    demand = MathUtil.applyDeadband(demand, Constants.OI.JOYSTICKS_DEADBAND_VALUE);

    arm.setSpeed(demand * Constants.Arm.CONTROLLER_MULTIPLIER);
  }

  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
