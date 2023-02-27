package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationPID extends CommandBase {
  private final Drivetrain drivetrain;
  private final double POSITION_TOLERANCE = Constants.Autos.ChargeStationBalance.PID.POSITION_TOLERANCE;
  private final double VELOCITY_TOLERANCE = Constants.Autos.ChargeStationBalance.PID.VELOCITY_TOLERANCE;
  private final double TARGET_ANGLE = Constants.Autos.ChargeStationBalance.PID.TARGET_ANGLE;


  private final PIDController pidController = new PIDController(
      Constants.Autos.ChargeStationBalance.PID.KP,
      Constants.Autos.ChargeStationBalance.PID.KI,
      Constants.Autos.ChargeStationBalance.PID.KD
  );

  public BalanceOnChargeStationPID(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    pidController.setSetpoint(TARGET_ANGLE);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double pitch = drivetrain.getPitch();
    double pidResult = pidController.calculate(pitch);
    drivetrain.setSpeed(pidResult, pidResult);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setSpeed(0, 0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
