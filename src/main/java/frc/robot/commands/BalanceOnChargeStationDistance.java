package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BalanceOnChargeStationDistance extends SequentialCommandGroup {
  public BalanceOnChargeStationDistance() {
    addCommands(
      new DriveToDistance(null),
      new WaitCommand(1),
      new BalanceOnChargeStationAuto(null)
    );
  }
}
