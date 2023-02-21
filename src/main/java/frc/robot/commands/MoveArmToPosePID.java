// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmToPosePID extends CommandBase {
  private Arm arm;
  private final PIDController pid = new PIDController(Constants.Arm.KP, Constants.Arm.KI, Constants.Arm.KD);
  private double targetPosition;

  /** Creates a new ArmPID. */
  public MoveArmToPosePID(double targetPosition, Arm arm) {
    this.targetPosition = targetPosition;
    this.arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(Constants.Arm.TOLERANCE_POSTION,Constants.Arm.TOLERANCE_VELOCITY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPostion = arm.getAngle();
    double pidResult = pid.calculate(currentPostion / 360, targetPosition / 360);
    arm.setSpeed(pidResult);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
