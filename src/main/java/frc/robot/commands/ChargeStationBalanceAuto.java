package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ChargeStationBalanceAuto extends CommandBase {
    Drivetrain drivetrain;

    public ChargeStationBalanceAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.set(Constants.GetOnChargeStationAuto.DRIVE_SPEED, Constants.GetOnChargeStationAuto.DRIVE_SPEED);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(0, 0);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getPitch() > Constants.GetOnChargeStationAuto.FINISH_ANGLE;
    }
}