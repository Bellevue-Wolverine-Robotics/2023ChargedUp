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

  // private CvSink m_cvSink = CameraServer.getVideo();
  //private CvSource m_outputStream = CameraServer.putVideo("Blur", 1920, 1080);
 // private CvSource m_outputStream = CameraServer.putVideo("Blur", 640, 480);
  private CvSink m_cvSink;
  private CvSource m_outputStream;

  private Mat mat;

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

    m_autoChooser.setDefaultOption("One Cone & Leave Community", AutoEnum.ONE_CONE_LEAVE_COMMUNITY);
    m_autoChooser.addOption("One Cone & Balance Charge Station", AutoEnum.ONE_CONE_BALANCE_CHARGE_STATION);
    // m_autoChooser.addOption("Charge Station", "ChargeStation");
    // m_autoChooser.addOption("Path Weaver", "PathWeaver");
    // m_autoChooser.addOption("Calibrate Arm", "CalibrateArm");

    SmartDashboard.putData("Auto Chooser", m_autoChooser);


    UsbCamera m_camera = CameraServer.startAutomaticCapture();
    m_camera.setResolution(640, 480);
    m_camera.setFPS(15);

    m_cvSink = CameraServer.getVideo();
    m_cvSink = CameraServer.getVideo();
    m_outputStream = CameraServer.putVideo("Rectangle", 1920, 1080);
    
    mat = new Mat();


    /*m_source = new Mat();
    m_output = new Mat();*/
    // m_visionThread = new Thread(
    //   () -> {
    //     UsbCamera m_camera = CameraServer.startAutomaticCapture();
    //     camera.set_resolution(640, 480);
    //     camera.setFPS(15)
    //   }
    // )
    // }
    
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

    
    
    
    if (m_cvSink.grabFrame(mat) == 0) {
       return;
    }


    Imgproc.rectangle(mat, new Point(100, 100), new Point(1080, 1080), new Scalar(255, 255, 255), 5);
    
    // Imgproc.cvtColor(m_source, m_output, Imgproc.COLOR_BGR2GRAY);
    m_outputStream.putFrame(mat);
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
