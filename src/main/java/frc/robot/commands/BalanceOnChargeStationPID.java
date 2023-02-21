// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationPID extends CommandBase {
  Drivetrain drivetrain;
  double pidResult;
  PIDController pidController = new PIDController(Constants.BalanceOnChargeStationPID.KP, Constants.BalanceOnChargeStationPID.KI, Constants.BalanceOnChargeStationPID.KD);

  public BalanceOnChargeStationPID(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pidResult = pidController.calculate(drivetrain.getPitch(), Constants.BalanceOnChargeStationPID.FINISH_ANGLE);
    drivetrain.set(pidResult, pidResult);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.set(0, 0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
