// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase getAutoCommand(Drivetrain drivetrain, Arm arm) {
    // return Commands.deadline(
    //     new GetOnChargeStationAuto(drivetrain)
    //         .andThen(new BalanceOnChargeStationAuto(drivetrain),
    //     new MoveArmToPosePID(Constants.Arm.POSTION_REST, arm))
    // );
    return new GetOnChargeStationAuto(drivetrain).andThen(new BalanceOnChargeStationAuto(drivetrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
