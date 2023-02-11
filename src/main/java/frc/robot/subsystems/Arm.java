package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.Arm.motorID, MotorType.kBrushless);
    private Arm instance = null;
    
    private Arm() {
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setSpeed(double speedDemand) {
        motor.set(speedDemand);
    }

    public Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }
}
