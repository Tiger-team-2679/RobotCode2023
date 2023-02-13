// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  Drivetrain drivetrain = Drivetrain.getInstance();
  Intake intake = Intake.getInstance();

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {
    OI.driverController.a().onTrue(new InstantCommand(() -> drivetrain.setVelocity(0.4, 0.4)));
    OI.driverController.b().onTrue(new InstantCommand(() -> drivetrain.setVelocity(0, 0)));
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
