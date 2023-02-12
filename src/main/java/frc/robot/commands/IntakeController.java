package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

public class IntakeController extends CommandBase {
  private Intake intake;


  public IntakeController(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double leftTrigger = OI.driverController.getLeftTriggerAxis();
    double rightTrigger = OI.driverController.getRightTriggerAxis();
    
    double speed = leftTrigger > rightTrigger ? -leftTrigger : rightTrigger;

    intake.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
