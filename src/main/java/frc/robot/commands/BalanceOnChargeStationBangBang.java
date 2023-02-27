package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationBangBang extends CommandBase {
  private final Drivetrain drivetrain;

  private final boolean IS_REVERSED = Constants.Autos.ChargeStationBalance.IS_REVERSED;
  private final double DRIVE_SPEED_FORWARD = Constants.Autos.ChargeStationBalance.BangBang.DRIVE_SPEED_FORWARD;
  private final double DRIVE_SPEED_BACKWARDS = Constants.Autos.ChargeStationBalance.BangBang.DRIVE_SPEED_BACKWARDS;
  private final double MISTAKE_ANGLE = Constants.Autos.ChargeStationBalance.BangBang.MISTAKE_ANGLE;

  public BalanceOnChargeStationBangBang(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double pitch =  (IS_REVERSED ? -1 : 1) * drivetrain.getPitch();
    double forwardSpeed = (IS_REVERSED ? -1 : 1) * DRIVE_SPEED_FORWARD;
    double backwardsSpeed = (IS_REVERSED ? 1 : -1) * DRIVE_SPEED_BACKWARDS;

    if(pitch > MISTAKE_ANGLE)
      drivetrain.setSpeed(forwardSpeed, forwardSpeed);
    else if(pitch < -MISTAKE_ANGLE)
      drivetrain.setSpeed(backwardsSpeed, backwardsSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setSpeed(0, 0);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrain.getPitch()) < Constants.Autos.ChargeStationBalance.BangBang.FINISH_ANGLE);
  }
}
