package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArmToPosePID extends CommandBase {
  private final Arm arm;
  private final PIDController pidController;

  public MoveArmToPosePID(double targetPosition, Arm arm, double kp, double kd, double ki) {
    this.arm = arm;
    addRequirements(arm);

    pidController = new PIDController(kp, ki, kd);
    pidController.setSetpoint(targetPosition / 360);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double currentPosition = arm.getAngle();
    double pidResult = pidController.calculate(currentPosition / 360);
    arm.setSpeed(pidResult);
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
