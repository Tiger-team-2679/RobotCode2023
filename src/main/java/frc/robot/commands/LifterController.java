package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lifter;

public class LifterController extends CommandBase {
  private final Lifter lifter;
  private final DoubleSupplier demandSupplier;

  public LifterController(Lifter lifter, DoubleSupplier demandSupplier) {
    this.lifter = lifter;
    addRequirements(lifter);
    this.demandSupplier = demandSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    lifter.setSpeed(demandSupplier.getAsDouble() * Constants.Lifter.MULTIPLIER);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
