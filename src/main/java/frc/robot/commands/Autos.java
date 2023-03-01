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
              .withTimeout(Constants.Autos.ReleaseCube.ARM_MOVE_TO_REST_TIME_SECONDS
            )
    );
  }

  public static Command driveBackwardsOutsideCommunity(Drivetrain drivetrain) {
    return new DriveToDistance(drivetrain, -Constants.Autos.DriveBackwardsOutsideCommunity.DISTANCE_METERS);
  }

  public static Command balanceChargeStation(Drivetrain drivetrain, Arm arm) {
    return new GetOnChargeStationAuto(drivetrain).andThen(new BalanceOnChargeStationPID(drivetrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
