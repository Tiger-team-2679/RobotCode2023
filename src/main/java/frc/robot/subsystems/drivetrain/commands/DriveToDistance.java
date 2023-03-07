package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveToDistance extends CommandBase {
  private final Drivetrain drivetrain;
  private final PIDController pidControllerLeft = new PIDController(
          Constants.Autos.DriveToDistance.KP,
          Constants.Autos.DriveToDistance.KI,
          Constants.Autos.DriveToDistance.KD);
  private final PIDController pidControllerRight = new PIDController(
          Constants.Autos.DriveToDistance.KP,
          Constants.Autos.DriveToDistance.KI,
          Constants.Autos.DriveToDistance.KD);
  private final double POSITION_TOLERANCE = Constants.Autos.DriveToDistance.POSITION_TOLERANCE;
  private final double VELOCITY_TOLERANCE = Constants.Autos.DriveToDistance.VELOCITY_TOLERANCE;

  public DriveToDistance(Drivetrain drivetrain, double meters) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    pidControllerLeft.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    pidControllerLeft.setSetpoint(meters + drivetrain.getLeftDistanceMeters());
    pidControllerRight.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    pidControllerRight.setSetpoint(meters + drivetrain.getRightDistanceMeters());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double pidResultLeft = pidControllerLeft.calculate(drivetrain.getLeftDistanceMeters());
    double pidResultRight = pidControllerRight.calculate(drivetrain.getRightDistanceMeters());

    drivetrain.setSpeed(pidResultLeft, pidResultRight);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setSpeed(0, 0);
  }

  @Override
  public boolean isFinished() {
    return pidControllerLeft.atSetpoint() && pidControllerRight.atSetpoint();
  }
}
