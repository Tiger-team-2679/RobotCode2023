package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.ArmController;
import frc.robot.subsystems.arm.commands.MoveArmToPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.ArcadeDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Arm arm = Arm.getInstance();
    public final CommandXboxController driverController = new CommandXboxController(Constants.OI.DRIVER_PORT);
    public final CommandXboxController operatorController = new CommandXboxController(Constants.OI.OPERATOR_PORT);

    private interface CommandSupplier {
        Command getCommand();
    }

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
                        arm));

        secondAutoCommandChooser.addOption(
                "Drive Backwards Outside Community and turn 180 degrease",
                () -> Autos.driveBackwardsOutsideCommunity(drivetrain,true));

        secondAutoCommandChooser.addOption(
                "Drive Backwards Outside Community",
                () -> Autos.driveBackwardsOutsideCommunity(drivetrain, false));

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
                () -> driverController.rightBumper().getAsBoolean()));

        // operator

        intake.setDefaultCommand(new IntakeController(
                intake,
                operatorController::getRightTriggerAxis,
                operatorController::getLeftTriggerAxis));

        arm.setDefaultCommand(new ArmController(arm,
                () -> -operatorController.getLeftY(),
                () -> -operatorController.getRightY()));

        operatorController.a().onTrue(new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_REST_SHOULDER,
                ArmConstants.ANGLE_REST_ELBOW));
        operatorController.b().onTrue(new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_FIRST_SHOULDER,
                ArmConstants.ANGLE_FIRST_ELBOW));
        operatorController.x().onTrue(new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_SECOND_SHOULDER,
                ArmConstants.ANGLE_SECOND_ELBOW));
        operatorController.y().onTrue(new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_THIRD_SHOULDER,
                ArmConstants.ANGLE_THIRD_ELBOW));

        operatorController.leftBumper().onTrue(new InstantCommand(
                () -> arm.setEmergencyMode(!arm.getEmergencyMode())));
        operatorController.rightBumper().whileTrue(
                new InstantCommand(() -> {
                    arm.setSpeedShoulder(0);
                    arm.setSpeedElbow(0);
                }, arm).repeatedly());
    }

    public Command getAutonomousCommand() {
        return new InstantCommand(() -> drivetrain.resetPitch())
                .andThen(firstAutoCommandChooser.getSelected().getCommand())
                .andThen(secondAutoCommandChooser.getSelected().getCommand());
    }
}
