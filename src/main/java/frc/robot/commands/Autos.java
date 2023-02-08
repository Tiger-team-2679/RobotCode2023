// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command getAutoCommand(Drivetrain drivetrain) {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));

    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
      drivetrain::getPose2d,
      drivetrain::resetPose2d,
      new RamseteController(),
      new DifferentialDriveKinematics(0.45),
      new SimpleMotorFeedforward(1.23, 1.22, 0.29),
      () -> new DifferentialDriveWheelSpeeds(drivetrain.getLeftEncoder().getRate(), drivetrain.getRightEncoder().getRate()),
      new PIDConstants(0, 0, 0),
      (left, right) -> drivetrain.set(left * 0.1, right * 0.1),
      null,
      drivetrain);
    return autoBuilder.followPath(examplePath);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
