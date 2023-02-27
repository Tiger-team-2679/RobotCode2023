package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public final class Autos {
  public static Command releaseConeAndDriveBackwards(Intake intake, Drivetrain drivetrain) {
    return new ConeReleaseAuto(intake).andThen(new DriveToDistance(drivetrain, -Constants.Autos.ReleaseConeAndDriveBackwards.DRIVE_DISTANCE));
  };

  public enum BalancingOptions {
    BANG_BANG,
    PID,
    DISTANCE
  }

  private static Command getBalancingCommand(Drivetrain drivetrain, BalancingOptions balancingOption) {
    switch(balancingOption){
      case BANG_BANG:
        return new BalanceOnChargeStationBangBang(drivetrain);
      case PID:
        return new BalanceOnChargeStationPID(drivetrain);
      case DISTANCE:
        return new BalanceOnChargeStationDistance(drivetrain);
    }
    return new InstantCommand();
  };

  public static Command balanceChargeStation(Drivetrain drivetrain, Arm arm, BalancingOptions balancingOption) {
    return Commands.deadline(
            new GetOnChargeStationAuto(drivetrain)
                    .andThen(getBalancingCommand(drivetrain, balancingOption)),
            new MoveArmToPosePID(Constants.Arm.POSITION_REST, arm, Constants.Arm.KP_REST, Constants.Arm.KD_REST, Constants.Arm.KI_REST)
    );
  }

  public static Command releaseConeAndBalanceChargeStation(Intake intake, Drivetrain drivetrain, Arm arm, BalancingOptions balancingOption) {
    return new ConeReleaseAuto(intake).andThen(balanceChargeStation(drivetrain, arm, balancingOption));
  };

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
