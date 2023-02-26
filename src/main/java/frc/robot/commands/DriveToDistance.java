
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistance extends CommandBase {
  private final Drivetrain drivetrain;
  private final PIDController pidController = new PIDController(Constants.DriveToDistance.KP, Constants.DriveToDistance.KI, Constants.DriveToDistance.KD);
  private final double distance;

  public DriveToDistance(Drivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.distance = distance;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double pidResultRight = pidController.calculate(drivetrain.getRightDistanceMeters(), distance);
    double pidResultLeft = pidController.calculate(drivetrain.getLeftDistanceMeters(), distance);

    SmartDashboard.putNumber("left", drivetrain.getLeftDistanceMeters());
    SmartDashboard.putNumber("right", drivetrain.getRightDistanceMeters());

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
