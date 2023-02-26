// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationPID extends CommandBase {
  private Drivetrain drivetrain;
  private double pidResult;
  private double pitch;
  private final boolean isReversed = Constants.chargeStationBalance.IS_REVERSED;
  private PIDController pidController = new PIDController(Constants.BalanceOnChargeStationPID.KP, Constants.BalanceOnChargeStationPID.KI, Constants.BalanceOnChargeStationPID.KD);

  public BalanceOnChargeStationPID(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    pidController.setTolerance(5, 0.5);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pitch = (isReversed ? -1 : 1) * drivetrain.getPitch();

    pidResult = pidController.calculate(pitch, Constants.BalanceOnChargeStationPID.FINISH_ANGLE);
    drivetrain.setSpeed(pidResult, pidResult);
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
