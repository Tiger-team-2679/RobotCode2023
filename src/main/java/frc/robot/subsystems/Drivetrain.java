package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;

public class Drivetrain extends SubsystemBase {
  private final TalonSRX leftMotor = new TalonSRX(Constants.Drivetrain.leftID);
  private final TalonSRX leftMotorFollower = new TalonSRX(Constants.Drivetrain.leftFollowerID);
  private final TalonSRX rightMotor = new TalonSRX(Constants.Drivetrain.rightID);
  private final TalonSRX rightMotorFollower = new TalonSRX(Constants.Drivetrain.rightFollowerID);
  private final PigeonIMU imu = new PigeonIMU(leftMotorFollower);
  private final Encoder leftEncoder = new Encoder(0, 1);
  private final Encoder rightEncoder = new Encoder(3, 2);

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    new Rotation2d(Math.toRadians(imu.getYaw())),
    leftEncoder.getDistance(),
    rightEncoder.getDistance()
  );

  private static Drivetrain instance = null;

  /** Creates a new Drivetrain. */
  private Drivetrain() {
    leftMotorFollower.follow(leftMotor);
    rightMotorFollower.follow(rightMotor);
    setDefaultCommand(new ArcadeDrive(this));

    double wheelRadiusInMeters = 0.076;
    int pulsesInRound = 2048;

    double distancePerRound = wheelRadiusInMeters * 2 * Math.PI;
    double roundsPerPules = 1 / (double) pulsesInRound;

    double distancePerPules = distancePerRound * roundsPerPules;

    rightEncoder.setDistancePerPulse(distancePerPules);
    leftEncoder.setDistancePerPulse(distancePerPules);
    rightEncoder.reset();
    leftEncoder.reset();

    imu.setYaw(0);
  }

  public void set(double leftDemand, double rightDemand) {
    SmartDashboard.putNumber("right", rightDemand);
    SmartDashboard.putNumber("left", leftDemand);

    leftMotor.set(ControlMode.PercentOutput, leftDemand);
    rightMotor.set(ControlMode.PercentOutput, -rightDemand);
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

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public void resetPose2d(Pose2d pose2d) {
    imu.setYaw(pose2d.getRotation().getDegrees());
    rightEncoder.reset();
    leftEncoder.reset();
    odometry.resetPosition(pose2d.getRotation(), 0, 0, pose2d);
  }

  @Override
  public void periodic() {
    odometry.update(    
      new Rotation2d(Math.toRadians(imu.getYaw())),
      leftEncoder.getDistance(),
      rightEncoder.getDistance()
    );

    SmartDashboard.putNumber("Encoder left", leftEncoder.getDistance());
    SmartDashboard.putNumber("Encoder right", rightEncoder.getDistance());
    SmartDashboard.putNumber("yaw", imu.getYaw());
  }

  public static Drivetrain getInstance() {
    if(instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }
}
