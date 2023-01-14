// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxRelativeEncoder;
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

  private DifferentialDriveOdometry m_odometry; 
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    this.m_leftFront.restoreFactoryDefaults();
    this.m_leftBack.restoreFactoryDefaults();
    this.m_rightFront.restoreFactoryDefaults();
    this.m_rightBack.restoreFactoryDefaults();

    this.m_leftGroup.setInverted(true);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_leftEncoder.setPositionConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS);
    m_rightEncoder.setPositionConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS);

    // WPILIB expects encoder rate to be in M/S while REV returns M/Min
    m_leftEncoder.setVelocityConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / 60);
    m_rightEncoder.setVelocityConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / 60);

    m_gyro.reset();
    m_gyro.calibrate();

    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(), 
      m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
      new Pose2d());

    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), new Pose2d());
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

  @Override
  public void periodic() {
    Pose2d pose = m_odometry.update(m_gyro.getRotation2d(), this.m_leftEncoder.getPosition(), this.m_rightEncoder.getPosition());


  } 

  public void arcadeDrive(double x, double y){
    this.m_drive.arcadeDrive(x, y);
  }

  public void arcadeDriveSquared(double x, double y){
    this.m_drive.arcadeDrive(x, y, true);
  }

  public void tankDrive(double y1, double y2){
    this.m_drive.tankDrive(y1, y2);
  }

  public void testDrive()
  {
    System.out.println("TEST DRIVE");
    m_leftBack.set(0.1);
    m_rightBack.set(0.1);
    m_leftFront.set(0.1);
    m_rightFront.set( 0.1);
  }


}
