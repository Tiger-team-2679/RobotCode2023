package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ConeReleaseAuto extends SequentialCommandGroup {
  public ConeReleaseAuto(Intake intake) {
    addCommands(
            new InstantCommand(() -> intake.setSpeed(Constants.Autos.ReleaseConeAndDriveBackwards.RELEASE_SPEED), intake),
            new WaitCommand(Constants.Autos.ReleaseConeAndDriveBackwards.RELEASE_TIME_SECONDS),
            new InstantCommand(() -> intake.setSpeed(0), intake)
    );
  }
}
