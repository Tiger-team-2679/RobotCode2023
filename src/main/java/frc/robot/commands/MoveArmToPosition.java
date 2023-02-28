package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmToPosition extends CommandBase {
  private final Arm arm;
  private final PIDController pidController;


  private PIDController getPidControllerForPosition(Positions position) {
    switch(position) {
      case REST:
        PIDController pidControllerRest
                = new PIDController(Constants.Arm.KP_REST, Constants.Arm.KI_REST, Constants.Arm.KD_REST);
        pidControllerRest.setSetpoint(Constants.Arm.ANGLE_REST / 360);
        return pidControllerRest;
      case FIRST:
        PIDController pidControllerFirst
                = new PIDController(Constants.Arm.KP_FIRST, Constants.Arm.KI_FIRST, Constants.Arm.KD_FIRST);
        pidControllerFirst.setSetpoint(Constants.Arm.ANGLE_FIRST / 360);
        return pidControllerFirst;
      case SECOND:
        PIDController pidControllerSecond
                = new PIDController(Constants.Arm.KD_SECOND, Constants.Arm.KD_SECOND, Constants.Arm.KD_SECOND);
        pidControllerSecond.setSetpoint(Constants.Arm.ANGLE_SECOND / 360);
        return pidControllerSecond;
      case THIRD:
        PIDController pidControllerThird
                = new PIDController(Constants.Arm.KP_THIRD, Constants.Arm.KI_THIRD, Constants.Arm.KD_THIRD);
        pidControllerThird.setSetpoint(Constants.Arm.ANGLE_THIRD / 360);
        return pidControllerThird;
    }
    return new PIDController(0, 0, 0);
  }

  public enum Positions {
    REST,
    FIRST,
    SECOND,
    THIRD
  }

  public MoveArmToPosition(Arm arm, Positions position) {
    this.arm = arm;
    addRequirements(arm);

    pidController = getPidControllerForPosition(position);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double currentPosition = arm.getAngle();
    double pidResult = pidController.calculate(currentPosition / 360);
    arm.setSpeed(pidResult);
  }

  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
