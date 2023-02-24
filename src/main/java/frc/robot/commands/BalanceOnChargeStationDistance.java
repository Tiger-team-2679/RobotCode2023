package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationDistance extends SequentialCommandGroup {
  public BalanceOnChargeStationDistance(Drivetrain drivetrain) {
    addCommands(
      new DriveToDistance(drivetrain, Constants.BalanceOnChargeStationDistance.DISTANCE),
      new WaitCommand(0.5),
      new BalanceOnChargeStationAuto(drivetrain)
    );
  }
}
