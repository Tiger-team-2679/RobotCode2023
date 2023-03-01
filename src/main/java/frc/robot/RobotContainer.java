// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
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
  private interface CommandSupplier { Command getCommand(); }
  private final SendableChooser<CommandSupplier> firstAutoCommandChooser = new SendableChooser<>();
  private final SendableChooser<CommandSupplier> secondAutoCommandChooser = new SendableChooser<>();

  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    configureBindings();
    firstAutoCommandChooser.setDefaultOption(
            "Release Cone",
            () -> Autos.releaseCone(intake));

    firstAutoCommandChooser.addOption(
            "Release Cube",
            () -> Autos.releaseCube(arm, intake));

    firstAutoCommandChooser.addOption(
            "None",
            () -> new InstantCommand());

    SmartDashboard.putData("First Auto Command", firstAutoCommandChooser);

    secondAutoCommandChooser.setDefaultOption(
            "Balance On Charge Station",
            () -> Autos.balanceChargeStation(
                    drivetrain,
                    arm
            )
    );

    secondAutoCommandChooser.addOption(
            "Drive Backwards Outside Community",
            () -> Autos.driveBackwardsOutsideCommunity(drivetrain)
    );

    secondAutoCommandChooser.addOption(
            "None",
            () -> new InstantCommand());


    SmartDashboard.putData("Second Auto Command", secondAutoCommandChooser);
}

  private void configureBindings() {
    // driver

    drivetrain.setDefaultCommand(new ArcadeDrive(
            drivetrain,
            () -> -driverController.getLeftY(),
            driverController::getRightX,
            () -> driverController.leftBumper().getAsBoolean(),
            () -> driverController.rightBumper().getAsBoolean()
    ));

    // operator

    intake.setDefaultCommand(new IntakeController(
        intake,
            operatorController::getRightTriggerAxis,
            operatorController::getLeftTriggerAxis));

    new Trigger(() -> MathUtil.applyDeadband(operatorController.getLeftY(), Constants.OI.JOYSTICKS_DEADBAND_VALUE) != 0)
        .whileTrue(new ArmController(
            arm,
            () -> -operatorController.getLeftY()));

    operatorController.a().onTrue(new MoveArmToPosition(arm, MoveArmToPosition.Positions.REST));
    operatorController.b().onTrue(new MoveArmToPosition(arm, MoveArmToPosition.Positions.FIRST));
    operatorController.x().onTrue(new MoveArmToPosition(arm, MoveArmToPosition.Positions.SECOND));
    operatorController.y().onTrue(new MoveArmToPosition(arm, MoveArmToPosition.Positions.THIRD));
    operatorController.leftBumper().onTrue(new InstantCommand(arm::resetEncoder));
    operatorController.rightBumper().onTrue(new InstantCommand(() -> arm.setSpeed(0), arm));
  }

  public Command getAutonomousCommand() {
    return firstAutoCommandChooser.getSelected().getCommand()
            .andThen(secondAutoCommandChooser.getSelected().getCommand());
  }
}
