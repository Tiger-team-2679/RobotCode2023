package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.MoveArmToPositionPID;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.BalanceOnChargeStation;
import frc.robot.subsystems.drivetrain.commands.DriveToDistance;
import frc.robot.subsystems.drivetrain.commands.GetOnChargeStation;
import frc.robot.subsystems.intake.Intake;

public final class Autos {
  public static Command releaseCone(Intake intake) {
    return new InstantCommand(() -> intake.setSpeed(Constants.Autos.ReleaseCone.RELEASE_SPEED), intake)
            .andThen(new WaitCommand(Constants.Autos.ReleaseCone.RELEASE_TIME_SECONDS))
            .andThen(new InstantCommand(() -> intake.setSpeed(0), intake));
  }

  public static Command releaseCube(Arm arm, Intake intake) {
    return new MoveArmToPositionPID(arm, MoveArmToPositionPID.Positions.SECOND)
            .withTimeout(Constants.Autos.ReleaseCube.ARM_MOVE_TO_SECOND_TIME_SECONDS)
            .andThen(new InstantCommand(() -> intake.setSpeed(Constants.Autos.ReleaseCube.RELEASE_SPEED), intake))
            .andThen(new WaitCommand(Constants.Autos.ReleaseCube.RELEASE_TIME_SECONDS))
            .andThen(new InstantCommand(() -> intake.setSpeed(0), intake)
            .andThen(
              new MoveArmToPositionPID(arm, MoveArmToPositionPID.Positions.REST))
              .withTimeout(Constants.Autos.ReleaseCube.ARM_MOVE_TO_REST_TIME_SECONDS
            )
    );
  }

  public static Command releaseCubeToThird(Arm arm, Intake intake) {
    return new MoveArmToPositionPID(arm, MoveArmToPositionPID.Positions.THIRD)
            .withTimeout(Constants.Autos.ReleaseCubeToThird.ARM_MOVE_TO_SECOND_TIME_SECONDS)
            .andThen(new InstantCommand(() -> intake.setSpeed(Constants.Autos.ReleaseCubeToThird.RELEASE_SPEED), intake))
            .andThen(new WaitCommand(Constants.Autos.ReleaseCubeToThird.RELEASE_TIME_SECONDS))
            .andThen(new InstantCommand(() -> intake.setSpeed(0), intake)
            .andThen(
              new MoveArmToPositionPID(arm, MoveArmToPositionPID.Positions.REST))
              .withTimeout(Constants.Autos.ReleaseCube.ARM_MOVE_TO_REST_TIME_SECONDS
            )
    );
  }

  public static Command driveBackwardsOutsideCommunity(Drivetrain drivetrain) {
    return new DriveToDistance(drivetrain, -Constants.Autos.DriveBackwardsOutsideCommunity.DISTANCE_METERS);
  }

  public static Command balanceChargeStation(Drivetrain drivetrain, Arm arm) {
    return new GetOnChargeStation(drivetrain).withTimeout(Constants.Autos.GetOnChargeStationAuto.TIMEOUT_SECONDS)
      .andThen(new BalanceOnChargeStation(drivetrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
