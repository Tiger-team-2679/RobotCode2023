package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationDistance extends SequentialCommandGroup {
  private final double DISTANCE_METERS = Constants.Autos.ChargeStationBalance.Distance.DISTANCE_METERS;
  private final boolean IS_REVERSED = Constants.Autos.ChargeStationBalance.IS_REVERSED;
  public BalanceOnChargeStationDistance(Drivetrain drivetrain) {
    addCommands(
      new DriveToDistance(drivetrain, (IS_REVERSED ? -1 : 1) * DISTANCE_METERS),
      new WaitCommand(0.5),
      new BalanceOnChargeStationBangBang(drivetrain)
    );
  }
}
