// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  Thread m_visionThread;

  private RobotContainer m_robotContainer;
  SendableChooser<AutoEnum> m_autoChooser = new SendableChooser<>();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    m_robotContainer = new RobotContainer();

    m_autoChooser.setDefaultOption("Score Mid & Leave Community", AutoEnum.ONE_CONE_LEAVE_COMMUNITY);
    m_autoChooser.addOption("Score Mid & Balance Charge Station", AutoEnum.ONE_CONE_BALANCE_CHARGE_STATION);
    m_autoChooser.addOption("Score Low & Leave Community", AutoEnum.ONE_CONE_SCORE_LOW_LEAVE_COMMUNITY);
    m_autoChooser.addOption("Score Low & Balance Charge Station", AutoEnum.ONE_CONE_SCORE_LOW_CHARGE_STATION);
    m_autoChooser.addOption("Score Mid & Don't Move", AutoEnum.SCORE_MID_DO_NOTHING);
    m_autoChooser.addOption("Score Low & Don't Move", AutoEnum.SCORE_LOW_DO_NOTHING);
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
    m_visionThread.setDaemon(true);
    m_visionThread.start();
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.onAuton();

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

    // if (m_testCommand != null) {
    //   m_testCommand.cancel();
    // }
    m_robotContainer.onTeleop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // System.out.println("test init");
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    teleopInit();

    // m_testCommand = m_robotContainer.getTestCommand();
    // m_testCommand.schedule();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    teleopPeriodic();
    if (m_robotContainer.getArmSubsystem().isSwitchClosed()) m_robotContainer.getArmSubsystem().resetArmEncoder();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
