// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Utils;
import frc.robot.subsystems.Arm;

public class ArmJoystick extends CommandBase {
  Arm arm;
  /** Creates a new ArmJoystick. */
  public ArmJoystick(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 
    -OI.driverController.getLeftY();

    speed = MathUtil.clamp(speed, -1.0, 1.0);
    speed = Utils.DeadBand(0.2,-0.2,speed);

    arm.setSpeed(speed * Constants.Arm.multiplierController);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
