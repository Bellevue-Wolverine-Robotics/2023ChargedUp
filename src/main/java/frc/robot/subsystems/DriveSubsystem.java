// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;

public class DriveSubsystem extends SubsystemBase {
  private Field2d m_field = new Field2d();

  private CANSparkMax m_leftBack = new CANSparkMax(CANConstants.LEFT_BACK, MotorType.kBrushless);
  private CANSparkMax m_leftFront = new CANSparkMax(CANConstants.LEFT_FRONT, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(CANConstants.RIGHT_FRONT, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(CANConstants.RIGHT_BACK, MotorType.kBrushless);

  private MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftBack);
  private MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightFront, m_rightBack);

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);


  REVPhysicsSim _revSimulation = new REVPhysicsSim();
  private RelativeEncoder m_leftEncoder =  m_leftFront.getEncoder();
  private RelativeEncoder m_rightEncoder = m_rightFront.getEncoder();



  //Sketchy sim stuff
  private EncoderSim m_leftEncoderSim = new EncoderSim(new Encoder(CANConstants.LEFT_BACK, CANConstants.LEFT_FRONT));
  private EncoderSim m_rightEncoderSim = new EncoderSim(new Encoder(CANConstants.RIGHT_FRONT, CANConstants.RIGHT_BACK));
  

  private ADXRS450_Gyro _ADXRS450_Gyro = new ADXRS450_Gyro();
  private Gyro m_gyro = _ADXRS450_Gyro;
  private ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(_ADXRS450_Gyro);



  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
  DCMotor.getNEO(2),              // 2 NEO motors on each side of the drivetrain.
  8.45,                             // 8.45:1 gearing reduction.
  7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
  30.0,                             // The mass of the robot is 60 kg.
  Units.inchesToMeters(3),            // The robot uses 3" radius wheels.
  0.7112,                  // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  
  

  private Accelerometer m_accelerometer = new BuiltInAccelerometer();

  private MedianFilter m_accelZFilter = new MedianFilter(50);
  private LinearFilter m_accelPitchFilter = LinearFilter.movingAverage(80);
  private MedianFilter m_accelYFilter = new MedianFilter(50);

  private DifferentialDriveOdometry m_odometry; 

  private AHRS m_imu = new AHRS(SPI.Port.kMXP);

  // private AHRS m_imu 

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

    m_leftFront.setSmartCurrentLimit(DriveConstants.STALL_LIMIT, DriveConstants.FREE_LIMIT);
    m_rightFront.setSmartCurrentLimit(DriveConstants.STALL_LIMIT, DriveConstants.FREE_LIMIT);
    m_leftBack.setSmartCurrentLimit(DriveConstants.STALL_LIMIT, DriveConstants.FREE_LIMIT);
    m_rightBack.setSmartCurrentLimit(DriveConstants.STALL_LIMIT, DriveConstants.FREE_LIMIT);

    m_rightFront.setInverted(true);
    m_rightBack.setInverted(true);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_leftEncoder.setPositionConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO);
    m_rightEncoder.setPositionConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO);

    // WPILIB expects encoder rate to be in M/S while REV returns M/Min
    m_leftEncoder.setVelocityConversionFactor((PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO) / 60);
    m_rightEncoder.setVelocityConversionFactor((PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO) / 60);


    float stallTorque = (float) 3.35;
    float freeSpeed = (float) 5874.0;
    _revSimulation.addSparkMax(m_leftBack, stallTorque, freeSpeed);
    _revSimulation.addSparkMax(m_rightBack, stallTorque, freeSpeed);
    _revSimulation.addSparkMax(m_leftFront, stallTorque, freeSpeed);
    _revSimulation.addSparkMax(m_rightFront, stallTorque, freeSpeed);


    m_imu.calibrate();

    m_gyro.reset();
    m_gyro.calibrate();

    m_odometry = new DifferentialDriveOdometry(
      m_imu.getRotation2d(), 
      m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
      new Pose2d());

    m_odometry.resetPosition(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), new Pose2d());
    
    SmartDashboard.putData("Field", m_field);



    SmartDashboard.putData("Reset Drive Pose", runOnce(this::resetPose));
  }
  
  public void simulationInit(){
    //some bs
  }

  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(m_leftGroup.get() * RobotController.getInputVoltage(),
    m_rightGroup.get() * RobotController.getInputVoltage());
  
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);
  
    // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());


    //so PID using odometry in commands are udpated.
    m_leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    m_rightEncoder.setPosition(m_driveSim.getRightPositionMeters());
    
  }

  

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts)
  {
    m_leftGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(rightVolts);

    m_drive.feed();
  }

  public void resetEncoders()
  {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    /* 
    shouldnt reset
    m_leftEncoderSim.setDistance(0);
    m_rightEncoderSim.setDistance(0);
    m_driveSim.setPose(new Pose2d());
    */
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

  public double getPitchDegreesYWeightedZ (){
    // double pitchDegrees = -Math.atan(m_accelerometer.getY() / m_accelerometer.getZ()) * 180/Math.PI;

    //int ySign = (int) Math.signum(m_accelerometer.getY());
  
    int ySign = (int) Math.signum(m_accelYFilter.calculate(m_accelerometer.getY()));

    
    //int ySign = picthUp()? 1: -1;

    double zAccel = MathUtil.clamp(getZAccelWeighted(), -1, 1);

    double pitchRadians = ySign * Math.atan(Math.sqrt(1 - Math.pow(zAccel, 2)) / zAccel);

    double pitchDegrees = -Math.toDegrees(pitchRadians);


    pitchDegrees = m_accelPitchFilter.calculate(pitchDegrees);

    return pitchDegrees;
  }

  


  /*  deal with later
  private Deque<Boolean> previous_pitches;
  private int cur_avg = 0;

  public boolean picthUp(){
    Boolean positive = Math.signum(m_accelerometer.getY()) == 1.0;
    previous_pitches.addFirst(positive);

    if(previous_pitches.size() > 100){
      if(previous_pitches.removeLast()){
        cur_avg--;
      }
      else{
        cur_avg++;
      }
    }


    if(positive){
      cur_avg++;
    }
    else{
      cur_avg--;
    }


    if(cur_avg > 0){
      return true;
    }
    else{
      return false;
    }
  
  }*/
  public double getPitchDegreesY() {
    // double pitchDegrees = -Math.atan(m_accelerometer.getY() / m_accelerometer.getZ()) * 180/Math.PI;

    int ySign = (int) Math.signum(m_accelerometer.getY());
    //int ySign = picthUp()? 1: -1;


    double zAccel = MathUtil.clamp(m_accelerometer.getZ(), -1, 1);


    double pitchRadians = ySign * Math.atan(Math.sqrt(1 - Math.pow(zAccel, 2)) / zAccel);

    double pitchDegrees = -Math.toDegrees(pitchRadians);

    return pitchDegrees;
  }

  public double getZAccelWeighted() {
    double zAccel = m_accelZFilter.calculate(m_accelerometer.getZ());

    return zAccel;
  }


  @Override
  public void periodic() {
    Pose2d pose = m_odometry.update(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());

    // imu pitch is roll, roll is pitch
    SmartDashboard.putNumber("Robot Roll", m_imu.getPitch());
    SmartDashboard.putNumber("Robot Heading", m_imu.getYaw());
    SmartDashboard.putNumber("Robot Pitch", m_imu.getRoll());

    //m_field.setRobotPose(m_odometry.getPoseMeters());


    //m_fieldApproximation.setRobotPose(m_poseEstimator.getEstimatedPosition());

  }

  public double getPitchDegrees()
  {
    // pointing up is pos, pointing down is neg
    
    // imu returning roll as pitch
    return -m_imu.getRoll();
  }

  public double getYaw()
  {
    return m_imu.getYaw();
  }

  public void resetPose()
  {
    resetEncoders();
    m_imu.reset();

    m_odometry.resetPosition(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), new Pose2d());

  }
  
  public void resetOdometry(Pose2d initialPose)
  {
    resetEncoders();
    
    m_odometry.resetPosition(
      m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), initialPose);
  }

  public void arcadeDrive(double xSpeed, double zRotation, boolean squared) {
    // SmartDashboard.putNumber("Arcade Drive xSpeed", xSpeed);
    // SmartDashboard.putNumber("Arcade Drive zRotation", zRotation);

    this.m_drive.arcadeDrive(xSpeed, zRotation);
    // m_drive.arcadeDrive(m_slewRateLimiter.calculate(xSpeed), m_slewRateLimiter.calculate(zRotation), m_squareInputs);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean squareInputs)
  {
    m_drive.curvatureDrive(xSpeed, zRotation, squareInputs);
  }

  // https://ewpratten.com/blog/joystick-to-voltage/
  public void semiConstantCurvatureDrive(double xSpeed, double zRotation)
  {
    double leftSpeed = 12 * (((xSpeed + Math.abs(xSpeed) * zRotation) + (xSpeed + zRotation)) / 2);
    double rightSpeed = 12 * (((xSpeed - Math.abs(xSpeed) * zRotation) + (xSpeed - zRotation)) / 2);

    double maxOutput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));

    if (maxOutput > 1.0)
    {
      leftSpeed /= maxOutput;
      rightSpeed /= maxOutput;

    }
    
    tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    this.m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void setMode(IdleMode mode) {
    m_leftFront.setIdleMode(mode);
    m_leftBack.setIdleMode(mode);
    m_rightFront.setIdleMode(mode);
    m_rightBack.setIdleMode(mode);
  }

  public void stopDriveTrain()
  {
    m_drive.tankDrive(0, 0);
  }

  public void resetImu()
  {
    m_imu.reset();
  }

public Integer add(int a, int b) {
    return a + b;
} 
}
