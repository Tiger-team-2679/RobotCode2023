package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    private final Drivetrain drivetrain;

    private final DoubleSupplier forwardDemandSupplier;
    private final DoubleSupplier backwardDemandSupplier;

    private final BooleanSupplier IsSensitiveForwardSupplier;
    private final BooleanSupplier IsSensitiveRotationSupplier;

    private final double forwardMultiplier = Constants.ArcadeDrive.FORWARD_MULTIPLIER;
    private final double sensitiveForwardMultiplier = Constants.ArcadeDrive.FORWARD_SENSITIVE_MULTIPLIER;
    private final double rotationMultiplier = Constants.ArcadeDrive.ROTATION_MULTIPLIER;
    private final double sensitiveRotationMultiplier = Constants.ArcadeDrive.ROTATION_SENSITIVE_MULTIPLIER;

    public ArcadeDrive(
            Drivetrain drivetrain,
            DoubleSupplier forwardDemandSupplier,
            DoubleSupplier backwardDemandSupplier,
            BooleanSupplier isSensitiveForwardSupplier,
            BooleanSupplier isSensitiveRotationSupplier
    ) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
        this.forwardDemandSupplier = forwardDemandSupplier;
        this.backwardDemandSupplier = backwardDemandSupplier;
        this.IsSensitiveForwardSupplier = isSensitiveForwardSupplier;
        this.IsSensitiveRotationSupplier = isSensitiveRotationSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forwardDemand = forwardDemandSupplier.getAsDouble();
        double rotationDemand = backwardDemandSupplier.getAsDouble();

        forwardDemand = MathUtil.applyDeadband(forwardDemand, 0.1);
        rotationDemand = MathUtil.applyDeadband(rotationDemand, 0.1);
        
        forwardDemand = MathUtil.clamp(forwardDemand, -1.0, 1.0);
        rotationDemand = MathUtil.clamp(rotationDemand, -1.0, 1.0);

         
        // forwardDemand *=  ? forwardMultiplier : sensitiveForwardMultiplier;
        // rotationDemand *= IsSensitiveRotationSupplier.getAsBoolean() ?  sensitiveRotationMultiplier :rotationMultiplier;

        if(IsSensitiveForwardSupplier.getAsBoolean())forwardDemand *=sensitiveForwardMultiplier;
        else forwardDemand *=forwardMultiplier;
        if(IsSensitiveRotationSupplier.getAsBoolean())rotationDemand *=sensitiveRotationMultiplier;
        else forwardDemand *=rotationMultiplier;

        SmartDashboard.putBoolean("forward",IsSensitiveForwardSupplier.getAsBoolean());
        SmartDashboard.putBoolean("back",IsSensitiveRotationSupplier.getAsBoolean());




        
        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        forwardDemand = Math.copySign(forwardDemand * forwardDemand, forwardDemand);
        rotationDemand = Math.copySign(rotationDemand * rotationDemand, rotationDemand);

        double leftSpeed;
        double rightSpeed;

        double maxInput = Math.copySign(Math.max(Math.abs(forwardDemand), Math.abs(rotationDemand)), forwardDemand);

        if (Double.compare(forwardDemand, 0.0) >= 0) {
            // First quadrant, else second quadrant
            if (Double.compare(rotationDemand, 0.0) >= 0) {
                leftSpeed = maxInput;
                rightSpeed = forwardDemand - rotationDemand;
            } else {
                leftSpeed = forwardDemand + rotationDemand;
                rightSpeed = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (Double.compare(rotationDemand, 0.0) >= 0) {
                leftSpeed = forwardDemand + rotationDemand;
                rightSpeed = maxInput;
            } else {
                leftSpeed = maxInput;
                rightSpeed = forwardDemand - rotationDemand;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
            leftSpeed /= maxMagnitude;
            rightSpeed /= maxMagnitude;
        }

        drivetrain.setSpeed(leftSpeed, rightSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
