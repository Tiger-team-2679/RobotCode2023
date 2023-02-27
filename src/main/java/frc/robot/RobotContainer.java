// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
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
  private interface CommandSupplier {
    Command getCommand();
  }
  private final SendableChooser<CommandSupplier> autoCommandChooser = new SendableChooser<>();
  private final SendableChooser<Autos.BalancingOptions> autoBalancingOptionChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    autoBalancingOptionChooser.setDefaultOption("Bang Bang", Autos.BalancingOptions.BANG_BANG);
    autoBalancingOptionChooser.addOption("PID", Autos.BalancingOptions.PID);
    autoBalancingOptionChooser.addOption("Distance", Autos.BalancingOptions.DISTANCE);

    autoCommandChooser.setDefaultOption(
            "Release cone and drive backward",
            () -> Autos.releaseConeAndDriveBackwards(intake, drivetrain));

    autoCommandChooser.addOption(
            "Balance on charge station",
            () -> Autos.balanceChargeStation(drivetrain, arm, autoBalancingOptionChooser.getSelected()));

    if(Constants.Autos.ChargeStationBalance.IS_REVERSED)
      autoCommandChooser.addOption(
              "Release cone and balance on charge station",
              () -> Autos.releaseConeAndBalanceChargeStation(intake, drivetrain, arm, autoBalancingOptionChooser.getSelected()));
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

    operatorController.y().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_THIRD_LEVEL, arm,Constants.Arm.KP_THIRD, Constants.Arm.KD_THIRD, Constants.Arm.KI_THIRD));
    operatorController.x().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_SECOND_LEVEL, arm, Constants.Arm.KP_SECOND, Constants.Arm.KD_SECOND, Constants.Arm.KI_SECOND));
    operatorController.b().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_FIRST_LEVEL, arm, Constants.Arm.KP_FIRST, Constants.Arm.KD_FIRST, Constants.Arm.KI_FIRST));
    operatorController.a().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_REST, arm, Constants.Arm.KP_REST, Constants.Arm.KD_REST, Constants.Arm.KI_REST));
    operatorController.leftBumper().onTrue(new InstantCommand(arm::resetEncoder));
    operatorController.rightBumper().onTrue(new InstantCommand(() -> arm.setSpeed(0)));
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected().getCommand();
  }
}