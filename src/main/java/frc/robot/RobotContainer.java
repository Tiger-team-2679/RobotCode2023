// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Autos;
import frc.robot.commands.ChargeStationBalanceAuto;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  Drivetrain drivetrain = Drivetrain.getInstance();
  Intake intake = Intake.getInstance();

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {
    OI.driverController.a().onTrue(new ChargeStationBalanceAuto(drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.getAutoCommand();
  }
}
