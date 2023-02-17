// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import java.lang.Math.*;

import frc.robot.Constants;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax m_leftBack = new CANSparkMax(CANConstants.LEFT_BACK, MotorType.kBrushless);
  private CANSparkMax m_leftFront = new CANSparkMax(CANConstants.LEFT_FRONT, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(CANConstants.RIGHT_FRONT, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(CANConstants.RIGHT_BACK, MotorType.kBrushless);

  private MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftBack);
  private MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightFront, m_rightBack);

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  private RelativeEncoder m_leftEncoder =  m_leftFront.getEncoder();
  private RelativeEncoder m_rightEncoder = m_rightFront.getEncoder();

  private Gyro m_gyro = new ADXRS450_Gyro();

  private Accelerometer m_accelerometer = new BuiltInAccelerometer();

  private LinearFilter m_pitchFilter = LinearFilter.movingAverage(10);
  // private KalmanFilter m_pitchKalmanFilter = new KalmanFilter<>(null, null, null, null, null, getGyroDegrees())

  private DifferentialDriveOdometry m_odometry; 
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    this.m_leftFront.restoreFactoryDefaults();
    this.m_leftBack.restoreFactoryDefaults();
    this.m_rightFront.restoreFactoryDefaults();
    this.m_rightBack.restoreFactoryDefaults();

    m_leftFront.setIdleMode(IdleMode.kBrake);
    m_leftBack.setIdleMode(IdleMode.kBrake);
    m_rightFront.setIdleMode(IdleMode.kBrake);
    m_rightBack.setIdleMode(IdleMode.kBrake);

    this.m_rightGroup.setInverted(true);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_leftEncoder.setPositionConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO);
    m_rightEncoder.setPositionConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO);

    // WPILIB expects encoder rate to be in M/S while REV returns M/Min
    m_leftEncoder.setVelocityConversionFactor((PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO) / 60);
    m_rightEncoder.setVelocityConversionFactor((PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO) / 60);

    m_gyro.reset();
    m_gyro.calibrate();

    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(), 
      m_leftEncoder.getPosition(), -m_rightEncoder.getPosition(),
      new Pose2d());

    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), -m_rightEncoder.getPosition(), new Pose2d());
    
    SmartDashboard.putData("Reset Drive Pose", new InstantCommand(this::resetPose, this));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getX(){
    return getPose().getX();
  }
  public double getY(){
    return getPose().getY();
  }

  public double getGyroDegrees()
  {
    return m_gyro.getAngle();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public double getPitchRadiansY()
  {
    return Math.atan(m_accelerometer.getY() / m_accelerometer.getZ());
  }
  public double getPitchDegreesY(){
    //double reWeight = 1.0/(Math.pow(m_accelerometer.getY(), 2.0) + Math.pow(m_accelerometer.getX(), 2.0));


    double pitchDegrees = -Math.atan(m_accelerometer.getY() / m_accelerometer.getZ()) * 180/Math.PI;
    // double ySign = Math.abs(m_accelerometer.getY()) / m_accelerometer.getY() ;
    // double zAccel = MathUtil.clamp(m_accelerometer.getZ(), -1, 1);

    // System.out.println("Zaccel: " + zAccel);
    // double pitchRadians = ySign * Math.atan(Math.sqrt(1 - Math.pow(zAccel, 2)) / zAccel);
    // System.out.println("PITCH_RADIANS: " + pitchRadians);

    // double pitchDegrees = -Math.toDegrees(pitchRadians);

    //System.out.println("ACCeleratyion" + MathUtil.clamp(m_accelerometer.getZ(), -1, 1));
    //double pitch = Math.acos(MathUtil.clamp(m_accelerometer.getZ(), -1, 1)) *  180/Math.PI;
   // double pitch = Math.asin(MathUtil.clamp(m_accelerometer.getX(), -1, 1)) *  180/Math.PI;

    //System.out.println(pitch);
    return pitchDegrees;
  }

  public double getPitchDegreesYWeighted() {
    return m_pitchFilter.calculate(getPitchDegreesY());
  }


  @Override
  public void periodic() {
    Pose2d pose = m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), -m_rightEncoder.getPosition());
        
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());
    SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());

    SmartDashboard.putNumber("Accelerometer X", m_accelerometer.getX());
    SmartDashboard.putNumber("Accelerometer Y", m_accelerometer.getY());
    SmartDashboard.putNumber("Accelerometer Z", m_accelerometer.getZ());

    //System.out.println("Accelerometer: (" + m_accelerometer.getX() + ", " + m_accelerometer.getY() + ", " + m_accelerometer.getZ()-1 + ")");
    SmartDashboard.putNumber("Pitch", this.getPitchDegreesY());
  } 

  public void resetPose()
  {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_gyro.reset();

    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), -m_rightEncoder.getPosition(), new Pose2d());
    
    System.out.println("reset");
    Pose2d pose = m_odometry.getPoseMeters();
    System.out.println("Pose: X(" + pose.getX() + ") Y(" + pose.getY() + ") Degrees: " + pose.getRotation().getDegrees());
  }

  public void arcadeDrive(double xSpeed, double zRotation){
    this.m_drive.arcadeDrive(xSpeed, zRotation);
  }

  public void arcadeDriveSquared(double xSpeed, double zRotation){
    this.m_drive.arcadeDrive(xSpeed, zRotation, true);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    System.out.println("Tank Drive: left(" + leftSpeed + ") right(" + rightSpeed + ")");
    this.m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void setMode(IdleMode mode) {
    m_leftFront.setIdleMode(mode);
    m_leftBack.setIdleMode(mode);
    m_rightFront.setIdleMode(mode);
    m_rightBack.setIdleMode(mode);
  }

  public void testDrive()
  {
    m_leftGroup.set(0.1);
    m_rightGroup.set(0.1);
  }

  public void stopDriveTrain()
  {
    m_drive.stopMotor();
  }
}
