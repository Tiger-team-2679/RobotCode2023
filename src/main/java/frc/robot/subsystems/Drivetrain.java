package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;

public class Drivetrain extends SubsystemBase {
  private final TalonSRX leftMotor = new TalonSRX(Constants.Drivetrain.LEFT_ID);
  private final TalonSRX leftMotorFollower = new TalonSRX(Constants.Drivetrain.LEFT_FOLLOWER_ID);
  private final TalonSRX rightMotor = new TalonSRX(Constants.Drivetrain.RIGHT_ID);
  private final TalonSRX rightMotorFollower = new TalonSRX(Constants.Drivetrain.RIGHT_FOLLOWER_ID);

  private final PigeonIMU imu = new PigeonIMU(leftMotorFollower);
  private final Encoder leftEncoder = new Encoder(Constants.Drivetrain.LEFT_ENCODER_CHANNEL_A, Constants.Drivetrain.LEFT_ENCODER_CHANNEL_B);
  private final Encoder rightEncoder = new Encoder(Constants.Drivetrain.RIGHT_ENCODER_CHANNEL_A, Constants.Drivetrain.RIGHT_ENCODER_CHANNEL_B);

  private static Drivetrain instance = null;

  /** Creates a new Drivetrain. */
  private Drivetrain() {
    setDefaultCommand(new ArcadeDrive(this));

    leftMotorFollower.follow(leftMotor);
    rightMotorFollower.follow(rightMotor);

    rightMotor.setInverted(true);
    rightMotorFollower.setInverted(true);

    rightMotor.setNeutralMode(NeutralMode.Brake);
    rightMotorFollower.setNeutralMode(NeutralMode.Brake);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    leftMotorFollower.setNeutralMode(NeutralMode.Brake);


    double wheelRadiusInMeters = 0.076;
    int pulsesInRound = 2048;

    double distancePerRound = wheelRadiusInMeters * 2 * Math.PI;
    double roundsPerPules = 1 / (double) pulsesInRound;

    double distancePerPules = distancePerRound * roundsPerPules;

    rightEncoder.setDistancePerPulse(distancePerPules);
    leftEncoder.setDistancePerPulse(distancePerPules);
  }

  public void set(double leftDemand, double rightDemand) {
    leftMotor.set(ControlMode.PercentOutput, leftDemand*0.3);
    rightMotor.set(ControlMode.PercentOutput, rightDemand*0.3);
  }

  public PigeonIMU getIMU() {
    return imu;
  }

  public Encoder getLeftEncoder() {
      return leftEncoder;
  }

  public Encoder getRightEncoder() {
      return rightEncoder;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Encoder Distance", rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Encoder Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("PigeonIMU Yaw", imu.getYaw());
  }

  public static Drivetrain getInstance() {
    if(instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }
}
