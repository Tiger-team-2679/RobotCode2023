// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationAuto extends CommandBase {
  private Drivetrain drivetrain;
  private final boolean isReversed = Constants.chargeStationBalance.IS_REVERSED;
  private final double forwardSpeed = Constants.BalanceOnChargeStationAuto.DRIVE_SPEED_FORWARD;
  private final double backwardsSpeed = -Constants.BalanceOnChargeStationAuto.DRIVE_SPEED_BACKWARDS;

  /** Creates a new BalanceOnChargeStationAuto. */
  public BalanceOnChargeStationAuto(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch =  (isReversed ? -1 : 1) * drivetrain.getPitch();

    if(pitch > Constants.BalanceOnChargeStationAuto.MISTAKE_ANGLE)
      drivetrain.setSpeed(forwardSpeed, forwardSpeed);
    else if(pitch < -Constants.BalanceOnChargeStationAuto.MISTAKE_ANGLE)
      drivetrain.setSpeed(backwardsSpeed, backwardsSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrain.getPitch()) < Constants.BalanceOnChargeStationAuto.FINISH_ANGLE);
  }
}
