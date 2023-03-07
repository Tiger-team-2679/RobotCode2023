package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class MoveArmToPositionPID extends CommandBase {
  private final Arm arm;
  private final PIDController pidController;


  private PIDController getPidControllerForPosition(Positions position) {
    switch(position) {
      case REST:
        PIDController pidControllerRest
                = new PIDController(ArmConstants.PID.KP_REST, ArmConstants.PID.KI_REST, ArmConstants.PID.KD_REST);
        pidControllerRest.setSetpoint(ArmConstants.ANGLE_REST / 360);
        return pidControllerRest;
      case FIRST:
        PIDController pidControllerFirst
                = new PIDController(ArmConstants.PID.KP_FIRST, ArmConstants.PID.KI_FIRST, ArmConstants.PID.KD_FIRST);
        pidControllerFirst.setSetpoint(ArmConstants.ANGLE_FIRST / 360);
        return pidControllerFirst;
      case SECOND:
        PIDController pidControllerSecond
                = new PIDController(ArmConstants.PID.KP_SECOND, ArmConstants.PID.KI_SECOND, ArmConstants.PID.KD_SECOND);
        pidControllerSecond.setSetpoint(ArmConstants.ANGLE_SECOND / 360);
        return pidControllerSecond;
      case THIRD:
        PIDController pidControllerThird
                = new PIDController(ArmConstants.PID.KP_THIRD, ArmConstants.PID.KI_THIRD, ArmConstants.PID.KD_THIRD);
        pidControllerThird.setSetpoint(ArmConstants.ANGLE_THIRD / 360);
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

  public MoveArmToPositionPID(Arm arm, Positions position) {
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
