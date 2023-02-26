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
import frc.robot.commands.BalanceOnChargeStationAuto;
import frc.robot.commands.BalanceOnChargeStationPID;
import frc.robot.commands.GetOnChargeStationAuto;
import frc.robot.commands.IntakeController;
import frc.robot.commands.MoveArmToPosePID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Arm arm = Arm.getInstance();
  public final CommandXboxController driverController = new CommandXboxController(Constants.OI.DRIVER_PORT);
  public final CommandXboxController opertatorController = new CommandXboxController(Constants.OI.OPERTATOR_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new ArcadeDrive(
        drivetrain,
        () -> -driverController.getLeftY(),
        () -> driverController.getRightX() * 0.7));

    intake.setDefaultCommand(new IntakeController(
        intake,
        () -> opertatorController.getRightTriggerAxis(),
        () -> opertatorController.getLeftTriggerAxis()));

      new Trigger(() -> MathUtil.applyDeadband(opertatorController.getLeftY(), Constants.OI.DEADBAND_VALUE) !=0)
        .whileTrue(new ArmController(
        arm,
        () -> -opertatorController.getLeftY()));

    opertatorController.y().onTrue(new MoveArmToPosePID(Constants.Arm.POSITION_THIRD_LEVEL, arm,Constants.Arm.KP_THIRD, Constants.Arm.KD_THIRD, Constants.Arm.KI_THIRD));
    opertatorController.x().onTrue(new MoveArmToPosePID(Constants.Arm.POSTION_SECOND_LEVEL, arm, Constants.Arm.KP_SECOND, Constants.Arm.KD_SECOND, Constants.Arm.KI_SECOND));
    opertatorController.b().onTrue(new MoveArmToPosePID(Constants.Arm.POSTION_FIRST_LEVEL, arm, Constants.Arm.KP_FIRST, Constants.Arm.KD_FIRST, Constants.Arm.KI_FIRST));
    opertatorController.a().onTrue(new MoveArmToPosePID(Constants.Arm.POSTION_REST, arm, Constants.Arm.KP_REST, Constants.Arm.KD_REST, Constants.Arm.KI_REST));
    opertatorController.leftBumper().onTrue(new InstantCommand(() -> arm.resetEncoder()));
  }

  public Command getAutonomousCommand() {
    return Autos.getAutoCommand(drivetrain, arm);
  }
}