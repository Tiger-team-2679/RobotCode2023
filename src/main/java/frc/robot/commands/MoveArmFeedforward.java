// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmFeedforward extends CommandBase {
  Arm arm ;
  double targetPosition,feedForwarCal;
  private final PIDController pid = new PIDController(Constants.Arm.KPF, Constants.Arm.KIF, Constants.Arm.KDF);
  ArmFeedforward feedforward = new ArmFeedforward(Constants.Arm.KS, Constants.Arm.KG, Constants.Arm.KV, Constants.Arm.KA);
  /** Creates a new ArmFeedforward. */
  public MoveArmFeedforward(double targetPosition,Arm arm) {
    this.targetPosition = targetPosition;
    this.arm = arm;
    feedForwarCal =feedforward.calculate(Math.toRadians(targetPosition),Constants.Arm.MAX_SPEED);
    addRequirements(this.arm);
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
    arm.setSpeed(pid.calculate(currentPostion, targetPosition) + feedForwarCal);
    SmartDashboard.putNumber("target", targetPosition);
    SmartDashboard.putBoolean("at pos", pid.atSetpoint());
    System.out.println("executeFF");


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
