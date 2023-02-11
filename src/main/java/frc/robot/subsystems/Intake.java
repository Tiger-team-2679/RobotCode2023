// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonSRX motor = new TalonSRX(Constants.Intake.MOTOR_ID);
    private static Intake instance = new Intake();

    private Intake() { }

    public static Intake getInstance() {
        return instance;
    }

    public void setSpeed(double demand) {
        motor.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
