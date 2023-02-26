// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmToPosePID extends CommandBase {
  private final Arm arm;
  private double targetPosition;
  private final PIDController pid;

  /** Creates a new ArmPID. */
  public MoveArmToPosePID(double targetPosition, Arm arm, double kp, double kd, double ki) {
    pid = new PIDController(kp, ki, kd);
    this.targetPosition = targetPosition;
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(Constants.Arm.TOLERANCE_POSITION ,Constants.Arm.TOLERANCE_VELOCITY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPostion = arm.getAngle();
    double pidResult = pid.calculate(currentPostion / 360, targetPosition / 360);
    arm.setSpeed(pidResult);
    SmartDashboard.putNumber("pid result ", pidResult);
    SmartDashboard.putBoolean("at set point ", pid.atSetpoint());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
