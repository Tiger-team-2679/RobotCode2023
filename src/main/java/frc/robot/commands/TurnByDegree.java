package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnByDegree extends CommandBase {
    private final Drivetrain drivetrain;
    private final PIDController pidController = new PIDController(
            Constants.Autos.TurnByAngle.KP,
            Constants.Autos.TurnByAngle.KI,
            Constants.Autos.TurnByAngle.KD);

    public TurnByDegree(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        pidController.setSetpoint(angle / 360);
        pidController.setTolerance(
                Constants.Autos.TurnByAngle.POSITION_TOLERANCE,
                Constants.Autos.TurnByAngle.VELOCITY_TOLERANCE
        );
    }

    @Override
    public void initialize() {
        drivetrain.setYaw(0);
    }

    @Override
    public void execute() {
        double currAngle = drivetrain.getYaw() / 360;
        double pidResult = pidController.calculate(currAngle);
        drivetrain.setSpeed(pidResult, -pidResult);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
