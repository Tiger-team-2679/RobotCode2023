package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    private final Drivetrain drivetrain;

    private final DoubleSupplier forwardDemandSupplier;
    private final DoubleSupplier backwardDemandSupplier;

    public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier forwardDemandSupplier, DoubleSupplier backwardDemandSupplier) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
        this.forwardDemandSupplier = forwardDemandSupplier;
        this.backwardDemandSupplier = backwardDemandSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forwardDemand = MathUtil.clamp(forwardDemandSupplier.getAsDouble(), -1.0, 1.0);
        double rotationDemand = MathUtil.clamp(backwardDemandSupplier.getAsDouble(), -1.0, 1.0);

        if((forwardDemand < 0.2 && forwardDemand > 0) || (forwardDemand > -0.2 && forwardDemand < 0))
            forwardDemand = 0;
        
        if((rotationDemand < 0.2 && rotationDemand > 0) || (rotationDemand > -0.2 && rotationDemand < 0))
            rotationDemand = 0;

        
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
