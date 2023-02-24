package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class GetOnChargeStationAuto extends CommandBase {
    private Drivetrain drivetrain;
    private boolean isReversed = Constants.chargeStationBalance.IS_REVERSED;
    private double driveSpeed = Constants.GetOnChargeStationAuto.DRIVE_SPEED * (isReversed ? -1 : 1);
    private double finishAngle = Constants.GetOnChargeStationAuto.FINISH_ANGLE;

    public GetOnChargeStationAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setSpeed(driveSpeed, driveSpeed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        return (isReversed ? -1 : 1) * drivetrain.getPitch() > finishAngle;
    }

}
