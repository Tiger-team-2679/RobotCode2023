// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Autos;
import frc.robot.commands.MoveArmToPosePID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  // Intake intake = Intake.getInstance();
  private Arm arm = Arm.getInstance();

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {
    //OI.driverController.y().onTrue(new MoveArmToPosePID(Constants.Arm.POSTION_FEEDER, arm));
    //OI.driverController.x().onTrue(new MoveArmToPosePID(Constants.Arm.POSTION_SECOND_LEVEL, arm));
    //OI.driverController.b().onTrue(new MoveArmToPosePID(Constants.Arm.POSTION_FIRST_LEVEL, arm));
    //OI.driverController.a().onTrue(new MoveArmToPosePID(Constants.Arm.POSTION_REST, arm));
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
