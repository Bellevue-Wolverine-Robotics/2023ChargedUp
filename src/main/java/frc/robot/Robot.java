// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ModesEnum.AutoEnum;
import frc.robot.Utils.Alert;
import frc.robot.Utils.Alert.AlertType;

  


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
sl */
public class Robot extends LoggedRobot  {
  private Command m_autonomousCommand;
  Thread m_visionThread;
  
  private static final String defaultAuto = "Default";
  private static final String customAuto = "My Auto";
  private final LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>("Auto Choices");

  private RobotContainer m_robotContainer;
  SendableChooser<AutoEnum> m_autoChooser = new SendableChooser<>();
  private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.",
          AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.",
          AlertType.ERROR);
  private final Alert logOpenFileAlert = new Alert(
      "Failed to open log file. Data will NOT be logged.", AlertType.ERROR);
  private final Alert logWriteAlert =
      new Alert("Failed write to the log file. Data will NOT be logged.",
          AlertType.ERROR);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.",
          AlertType.WARNING);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    if (isReal()) {
      logger.addDataReceiver(new WPILOGWriter("C:\\Users\\team9\\OneDrive\\Desktop\\Log Telemetry")); 
      logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()

    // Start AdvantageKit logger
    logger.start();
    // Initialize auto chooser
    chooser.addDefaultOption("Default Auto", defaultAuto);
    chooser.addOption("My Auto", customAuto);


    m_robotContainer = new RobotContainer();

    m_autoChooser.setDefaultOption("Score Mid & Leave Community", AutoEnum.ONE_CONE_LEAVE_COMMUNITY);
    m_autoChooser.addOption("Score Mid & Balance Charge Station", AutoEnum.ONE_CONE_BALANCE_CHARGE_STATION);

    m_autoChooser.addOption("Score Low & Leave Community", AutoEnum.ONE_CONE_SCORE_LOW_LEAVE_COMMUNITY);
    m_autoChooser.addOption("Score Low & Balance Charge Station", AutoEnum.ONE_CONE_SCORE_LOW_CHARGE_STATION);

    m_autoChooser.addOption("Score Mid & Don't Move", AutoEnum.SCORE_MID_DO_NOTHING);
    m_autoChooser.addOption("Score Low & Don't Move", AutoEnum.SCORE_LOW_DO_NOTHING);

    m_autoChooser.addOption("Balance Only (No Score)", AutoEnum.LEAVE_COMMUNITY_ONLY);
    m_autoChooser.addOption("Leave Community Only (No Score)", AutoEnum.BALANCE_ONLY);

    m_autoChooser.addOption("Do Nothing", AutoEnum.DO_NOTHING);


    
    



    // m_autoChooser.addOption("Charge Station", "ChargeStation");
    // m_autoChooser.addOption("Path Weaver", "PathWeaver");
    // m_autoChooser.addOption("Calibrate Arm", "CalibrateArm");

    SmartDashboard.putData("Auto Chooser", m_autoChooser);




    m_visionThread =
    new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture();
          // Set the resolution
          camera.setResolution(640, 480);
          camera.setFPS(15);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Lebron POV", 640, 480);

          Mat m_source = new Mat();
          Mat m_output = new Mat();


          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(m_source) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            // Put a rectangle on the image
            if (cvSink.grabFrame(m_source) == 0) {
              return;
            }
        
        
            // Imgproc.rectangle(mat, new Point(100, 100), new Point(1080, 1080), new Scalar(255, 255, 255), 5);
            Imgproc.cvtColor(m_source, m_output, Imgproc.COLOR_BGR2GRAY);
            outputStream.putFrame(m_output);
          }
        });
    //m_visionThread.setDaemon(true);
    //m_visionThread.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    Logger.getInstance().recordOutput("ActiveCommands/Scheduler",
    NetworkTableInstance.getDefault()
      .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
      .getStringArray(new String[] {}));

    // Check logging faults


  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoChooser.getSelected());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    teleopInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
