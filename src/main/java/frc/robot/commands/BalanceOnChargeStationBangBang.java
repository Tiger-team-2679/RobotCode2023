package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationBangBang extends CommandBase {
  private final Drivetrain drivetrain;

  private final boolean IS_REVERSED = Constants.Autos.ChargeStationBalance.IS_REVERSED;
  private final double DRIVE_SPEED_FORWARD = Constants.Autos.ChargeStationBalance.BangBang.DRIVE_SPEED_FORWARD;
  private final double DRIVE_SPEED_BACKWARDS = Constants.Autos.ChargeStationBalance.BangBang.DRIVE_SPEED_BACKWARDS;
  private final double MISTAKE_ANGLE = Constants.Autos.ChargeStationBalance.BangBang.MISTAKE_ANGLE;
  private final double MISTAKE_ANGLE_BACKWARD = Constants.Autos.ChargeStationBalance.BangBang.MISTAKE_ANGLE_BACKWARD;
  private final double DISTANCE_TO_CENTER = Constants.Autos.ChargeStationBalance.BangBang.DISTANCE_TO_CENTER;

  private final PIDController leftDistancePID = new PIDController(
          Constants.Autos.DriveToDistance.KP * DRIVE_SPEED_FORWARD,
          Constants.Autos.DriveToDistance.KI,
          Constants.Autos.DriveToDistance.KD);
  private final PIDController rightDistancePID = new PIDController(
          Constants.Autos.DriveToDistance.KP * DRIVE_SPEED_FORWARD,
          Constants.Autos.DriveToDistance.KI,
          Constants.Autos.DriveToDistance.KD);

  private double lastAngle;
  private double velocity = 0;

  private double startLeftDistance;
  private double startRightDistance;



  public BalanceOnChargeStationBangBang(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    rightDistancePID.setSetpoint((IS_REVERSED ? -1 : 1) * DISTANCE_TO_CENTER);
    rightDistancePID.setSetpoint((IS_REVERSED ? -1 : 1) * DISTANCE_TO_CENTER);
  }

  @Override
  public void initialize() {
    startLeftDistance = drivetrain.getLeftDistanceMeters();
    startRightDistance = drivetrain.getRightDistanceMeters();
  }

  @Override
  public void execute() {
    double pitch = (IS_REVERSED ? -1 : 1) * drivetrain.getPitch();
    double forwardSpeed = (IS_REVERSED ? -1 : 1) * DRIVE_SPEED_FORWARD;
    double backwardsSpeed = (IS_REVERSED ? 1 : -1) * DRIVE_SPEED_BACKWARDS;

    velocity = (lastAngle - pitch) / 0.02;
    lastAngle = pitch;

    double pidResultLeft = leftDistancePID.calculate(drivetrain.getLeftDistanceMeters() - startLeftDistance);
    double pidResultRight = rightDistancePID.calculate(drivetrain.getRightDistanceMeters() - startRightDistance);

    if(pitch > MISTAKE_ANGLE)
      drivetrain.setSpeed(forwardSpeed + pidResultLeft, forwardSpeed + pidResultRight);
    else if(pitch < -MISTAKE_ANGLE_BACKWARD)
      drivetrain.setSpeed(backwardsSpeed - pidResultLeft, backwardsSpeed - pidResultRight);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setSpeed(0, 0);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrain.getPitch()) < Constants.Autos.ChargeStationBalance.BangBang.FINISH_ANGLE) 
        && velocity < Constants.Autos.ChargeStationBalance.BangBang.FINISH_VELOCITY;
  }
}
