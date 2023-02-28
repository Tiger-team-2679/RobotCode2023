package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public final class Autos {
  public static Command releaseCone(Intake intake) {
    return new InstantCommand(() -> intake.setSpeed(Constants.Autos.ReleaseCone.RELEASE_SPEED), intake)
            .andThen(new WaitCommand(Constants.Autos.ReleaseCone.RELEASE_TIME_SECONDS))
            .andThen(new InstantCommand(() -> intake.setSpeed(0), intake));
  }

  public static Command releaseCube(Arm arm, Intake intake) {
    return new MoveArmToPosition(arm, MoveArmToPosition.Positions.SECOND)
            .withTimeout(Constants.Autos.ReleaseCube.ARM_MOVE_TO_SECOND_TIME_SECONDS)
            .andThen(new InstantCommand(() -> intake.setSpeed(Constants.Autos.ReleaseCube.RELEASE_SPEED), intake))
            .andThen(new WaitCommand(Constants.Autos.ReleaseCube.RELEASE_TIME_SECONDS))
            .andThen(new InstantCommand(() -> intake.setSpeed(0), intake)
            .andThen(
              new MoveArmToPosition(arm, MoveArmToPosition.Positions.REST))
              .withTimeout(Constants.Autos.ReleaseCube.ARM_MOVE_TO_SECOND_TIME_SECONDS
            )
    );
  }

  public static Command driveBackwardsOutsideCommunity(Drivetrain drivetrain) {
    return new DriveToDistance(drivetrain, -Constants.Autos.DriveBackwardsOutsideCommunity.DISTANCE_METERS);
  }

  public enum BalancingOptions {
    BANG_BANG,
    PID,
    DISTANCE_BANG_BANG,
    DISTANCE_PID
  }

  private static Command getBalancingCommand(Drivetrain drivetrain, BalancingOptions balancingOption) {
    switch(balancingOption){
      case BANG_BANG:
        return new BalanceOnChargeStationBangBang(drivetrain);
      case PID:
        return new BalanceOnChargeStationPID(drivetrain);
      case DISTANCE_BANG_BANG:
        return new BalanceOnChargeStationDistance(drivetrain, false);
      case DISTANCE_PID:
        return new BalanceOnChargeStationDistance(drivetrain, true);
    }
    return new InstantCommand();
  };

  public static Command balanceChargeStation(Drivetrain drivetrain, Arm arm, BalancingOptions balancingOption, Timer timerFromAutoStart) {
    Command balancingCommand = getBalancingCommand(drivetrain, balancingOption);
    return Commands.deadline(
            new GetOnChargeStationAuto(drivetrain)
                    .andThen(balancingCommand)
                    .until(() -> timerFromAutoStart.hasElapsed(Constants.Autos.ChargeStationBalance.TIMEOUT_SECONDS_BEFORE_TURNING))
                    .andThen(new TurnByDegree(drivetrain, Constants.Autos.ChargeStationBalance.TURNING_ANGLE))
                    .unless(balancingCommand::isFinished),
            new MoveArmToPosition(arm, MoveArmToPosition.Positions.REST)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
