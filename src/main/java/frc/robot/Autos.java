package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.ArmPositionsCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.BalanceOnChargeStation;
import frc.robot.subsystems.drivetrain.commands.DriveToDistance;
import frc.robot.subsystems.drivetrain.commands.GetOnChargeStation;
import frc.robot.subsystems.drivetrain.commands.TurnByDegree;
import frc.robot.subsystems.intake.Intake;

public final class Autos {
  public static Command releaseCone(Intake intake) {
    return new InstantCommand(() -> intake.setSpeed(-Constants.Autos.ReleaseCone.RELEASE_SPEED), intake)
        .andThen(new WaitCommand(Constants.Autos.ReleaseCone.RELEASE_TIME_SECONDS))
        .andThen(new InstantCommand(() -> intake.setSpeed(0), intake));
  }

  public static Command releaseConeSecond(Arm arm, Intake intake){
    return ArmPositionsCommands.coneSecond(arm).withTimeout(Constants.Autos.releaseConeSecond.TIMEOUT_SECONDS_RAISE)
    .andThen(releaseCone(intake))
    .andThen(ArmPositionsCommands.rest(arm).withTimeout(Constants.Autos.releaseCubeSecond.TIMEOUT_SECONDS_LOWER));
  }

  public static Command releaseConeThird(Arm arm, Intake intake){
    return ArmPositionsCommands.coneThird(arm).withTimeout(Constants.Autos.releaseConeThird.TIMEOUT_SECONDS_RAISE)
    .andThen(releaseCone(intake))
    .andThen(ArmPositionsCommands.rest(arm).withTimeout(Constants.Autos.releaseConeThird.TIMEOUT_SECONDS_LOWER));
  }

  public static Command releaseCube(Intake intake) {
    return new InstantCommand(() -> intake.setSpeed(Constants.Autos.ReleaseCube.RELEASE_SPEED), intake)
        .andThen(new WaitCommand(Constants.Autos.ReleaseCube.RELEASE_TIME_SECONDS))
        .andThen(new InstantCommand(() -> intake.setSpeed(0), intake));
  }

  public static Command releaseCubeSecond(Arm arm, Intake intake){
    return ArmPositionsCommands.cubeSecond(arm).withTimeout(Constants.Autos.releaseCubeSecond.TIMEOUT_SECONDS_RAISE)
    .andThen(releaseCube(intake))
    .andThen(ArmPositionsCommands.rest(arm).withTimeout(Constants.Autos.releaseCubeSecond.TIMEOUT_SECONDS_LOWER));
  }

  public static Command releaseCubeThird(Arm arm, Intake intake){
    return ArmPositionsCommands.cubeThird(arm).withTimeout(Constants.Autos.releaseCubeThird.TIMEOUT_SECONDS_RAISE)
    .andThen(releaseCube(intake))
    .andThen(ArmPositionsCommands.rest(arm).withTimeout(Constants.Autos.releaseCubeThird.TIMEOUT_SECONDS_LOWER));
  }


  public static Command driveBackwardsOutsideCommunity(Drivetrain drivetrain, boolean turnOnFinish) {
    Command driveToDistanceCommand = new DriveToDistance(drivetrain,
        -Constants.Autos.DriveBackwardsOutsideCommunity.DISTANCE_METERS);

    return turnOnFinish
        ? driveToDistanceCommand.withTimeout(Constants.Autos.DriveBackwardsOutsideCommunity.TIMEOUT_SECONDS)
            .andThen(new TurnByDegree(drivetrain, Constants.Autos.DriveBackwardsOutsideCommunity.TURN_ENGLE))
        : driveToDistanceCommand.withTimeout(Constants.Autos.DriveBackwardsOutsideCommunity.TIMEOUT_SECONDS);
  }

  public static Command balanceChargeStation(Drivetrain drivetrain, Arm arm) {
    return new GetOnChargeStation(drivetrain).withTimeout(Constants.Autos.GetOnChargeStationAuto.TIMEOUT_SECONDS)
        .andThen(new BalanceOnChargeStation(drivetrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
