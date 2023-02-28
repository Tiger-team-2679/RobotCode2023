package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationDistance extends SequentialCommandGroup {
  private final double DISTANCE_METERS = Constants.Autos.ChargeStationBalance.Distance.DISTANCE_METERS;
  private final boolean IS_REVERSED = Constants.Autos.ChargeStationBalance.IS_REVERSED;
  public BalanceOnChargeStationDistance(Drivetrain drivetrain, boolean isPID) {
    addCommands(
      new DriveToDistance(drivetrain, (IS_REVERSED ? -1 : 1) * DISTANCE_METERS),
      new WaitCommand(Constants.Autos.ChargeStationBalance.Distance.WAIT_TIME_SECONDS),
            isPID
                    ? new BalanceOnChargeStationPID(drivetrain)
                    : new BalanceOnChargeStationBangBang(drivetrain)
    );
  }
}
