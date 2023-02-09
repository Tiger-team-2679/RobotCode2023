package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;

public class Drivetrain extends SubsystemBase {
  private final TalonSRX leftMotor = new TalonSRX(Constants.Drivetrain.leftID);
  private final TalonSRX leftMotorFollower = new TalonSRX(Constants.Drivetrain.leftFollowerID);
  private final TalonSRX rightMotor = new TalonSRX(Constants.Drivetrain.rightID);
  private final TalonSRX rightMotorFollower = new TalonSRX(Constants.Drivetrain.rightFollowerID);
  private final PigeonIMU imu = new PigeonIMU(leftMotorFollower);

  private static Drivetrain instance = null;

  /** Creates a new Drivetrain. */
  private Drivetrain() {
    setDefaultCommand(new ArcadeDrive(this));

    leftMotorFollower.follow(leftMotor);
    rightMotorFollower.follow(rightMotor);

    rightMotor.setInverted(true);
    rightMotorFollower.setInverted(true);
  }

  public void set(double leftDemand, double rightDemand) {
    leftMotor.set(ControlMode.PercentOutput, leftDemand);
    rightMotor.set(ControlMode.PercentOutput, rightDemand);
  }

  public PigeonIMU getIMU() {
    return imu;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Drivetrain getInstance() {
    if(instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }
}
