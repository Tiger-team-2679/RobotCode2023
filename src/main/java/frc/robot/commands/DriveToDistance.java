
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistance extends CommandBase {
  Drivetrain drivetrain;
  double pidResultRight;
  double pidResultLeft;
  PIDController pidController = new PIDController(Constants.DriveToDistance.KP, Constants.DriveToDistance.KI, Constants.DriveToDistance.KD);

  public DriveToDistance(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pidResultRight = pidController.calculate(drivetrain.getRightDistanceMeters(), Constants.DriveToDistance.FINISH_POINT);
    pidResultLeft = pidController.calculate(drivetrain.getLeftDistanceMeters(), Constants.DriveToDistance.FINISH_POINT);

    drivetrain.setSpeed(pidResultLeft, pidResultRight);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setSpeed(0, 0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
