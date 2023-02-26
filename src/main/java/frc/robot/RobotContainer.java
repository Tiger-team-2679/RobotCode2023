// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmController;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeController;
import frc.robot.commands.MoveArmToPosePID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Arm arm = Arm.getInstance();
  public final CommandXboxController driverController = new CommandXboxController(Constants.OI.DRIVER_PORT);
  public final CommandXboxController operatorController = new CommandXboxController(Constants.OI.OPERATOR_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new ArcadeDrive(
            drivetrain,
            () -> -driverController.getLeftY(),
            driverController::getRightX,
            () -> driverController.getHID().getLeftBumper(),
            () -> driverController.getHID().getRightBumper()
    ));

    intake.setDefaultCommand(new IntakeController(
        intake,
            operatorController::getRightTriggerAxis,
            operatorController::getLeftTriggerAxis));

      new Trigger(() -> MathUtil.applyDeadband(operatorController.getLeftY(), Constants.OI.DEADBAND_VALUE) != 0)
        .whileTrue(new ArmController(
        arm,
        () -> -operatorController.getLeftY()));

    operatorController.y().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_FEEDER, arm));
    operatorController.x().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_SECOND_LEVEL, arm));
    operatorController.b().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_FIRST_LEVEL, arm));
    operatorController.a().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_REST, arm));
    operatorController.leftBumper().onTrue(new InstantCommand(arm::resetEncoder));
  }

  public Command getAutonomousCommand() {
    return Autos.getAutoCommand(drivetrain, arm);
  }
}