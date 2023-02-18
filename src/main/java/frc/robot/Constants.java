// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int JOYSTICK_1 = 0;
    public static final int JOYSTICK_2 = 1;
    public static final int JOYSTICK_3 = 2;

    public static final int Xbox = 3;
  }

  public static class CANConstants {
      // CAN IDs
      public static final int LEFT_FRONT = 1;
      public static final int LEFT_BACK = 2;
      public static final int RIGHT_FRONT = 3;
      public static final int RIGHT_BACK = 4;
      public static final int ARM_SPARKMAX = 5;
      public static final int ARM_TALON = 12;
  }

  public static class PneumaticsConstants {
    public static final int INTAKE_FORWARD_CHANNEL = 0;
    public static final int INTAKE_REVERSE_CHANNEL = 1;
    public static final DoubleSolenoid.Value EXTEND_STATE = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value RETRACT_STATE = DoubleSolenoid.Value.kForward;
  }

  public static class PhysicalConstants {
    public static final double DRIVE_GEAR_RATIO = 8.45;
    public static final double WHEEEL_CIRCUMFERENCE_INCHES = 18.875;
    public static final double WHEEL_CIRCUMFERENCE_METERS =  Units.inchesToMeters(WHEEEL_CIRCUMFERENCE_INCHES);
    public static final int TALON_PULSES_PER_ROTATION = 4096;
  }

  public static class ButtonConstants {
    public static final int INTAKE_TOGGLE_BUTTON = 1;
    public static final int INTAKE_EXTEND_BUTTON = 5;
    public static final int INTAKE_RETRACT_BUTTON = 4;
    public static final int GRAB_CLOCKWISE_BUTTON = 6;
    public static final int GRAB_COUNTER_CLOCKWISE_BUTTON = 7;
    public static final int RESET_POSE_BUTTON = 8;

    public static final int ARM_LOW_BUTTON = 4;
    public static final int ARM_HIGH_BUTTON = 6;

    public static final int CHARGE_BALANCE_BUTTON = 7;

  }

  public static class FieldConstants {
    public static double RAMP_LENGTH_INCH = 26;
    public static double RAMP_LENGTH_METERS = RAMP_LENGTH_INCH*PhysicsConstants.INCH_TO_METER;
    public static double WHOLE_PLATFORM_LENGTH_INCH = 48;
    public static double WHOLE_PLATFORM_LENGTH_METER = WHOLE_PLATFORM_LENGTH_INCH*PhysicsConstants.INCH_TO_METER;
    
  }

  public static class PhysicsConstants{
    public static double INCH_TO_METER = 0.0254;
  }

  public static class DriveConstants {
    // TODO: SYSID CHANGE LATER
    public static final double VOLTS = 0;
    public static final double VOLTS_SECONDS_PER_METER = 0;
    public static final double VOLTS_SECONDS_SQUARED_PER_METER = 0;

    public static final double TRACKWIDTH_METERS = 0;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH_METERS);
    
    public static final double MAX_VOLTAGE = 10;

    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    // TUNE
    public static final double KP_DRIVE_VEL = 8;
  }
    //   // Inputs
    // public static final int QUICKTURN_BUTTON = 1;
  
    //   // Calculations        
    //   public static final int WHEEL_SIZE = 6;
    //   public static final double GEAR_RATIO = 1 / 10.71;
    //   // Calculate the position factor by calculating the circumference of the wheel times the gear ratio
    //   // From https://dev.revrobotics.com/sparkmax/software-resources/migrating-ctre-to-rev#change-units-from-rotations-to-inches
    //   public static final double POSITION_FACTOR = Math.PI * WHEEL_SIZE * GEAR_RATIO;
  
    //   // Calculate the ultrasonic's scale
    //   // Basically there is 293mV per 300mm from the ultrasonic
    //   // And we know that the max distance is 5,000mm we can just set up a proportion
    //   // 5000/x = 300/293
    //   // x = 1,500,000 / 293
    //   // Now we convert this to inches by multiplying by 0.0393701
    //   public static final double INCHES_PER_5V = (1_500_000 / 293) * 0.0393701;
  
  
    //   /*
    //    * Climb
    //    */
    //   // CAN IDs
    //   public static final int LONG_ARM_EXTEND_MOTOR = 6;
    //   public static final int LONG_ARM_PIVOT_MOTOR = 7;
    //   public static final int SMALL_ARM_1_MOTOR = 8;
    //   public static final int SMALL_ARM_2_MOTOR = 9;
  
    //   // Input
    //   public static final int LONG_ARM_PIVOT_BUTTON = 5;
    //   public static final int LONG_ARM_PIVOT_REVERSE_BUTTON = 3;
    //   public static final int LONG_ARM_EXTEND_BUTTON = 6;
    //   public static final int LONG_ARM_RETRACT_BUTTON = 4;
    //   public static final int LONG_ARM_OVERRIDE_BUTTON = 8;
    //   public static final int HOOKS_TOGGLE_BUTTON = 2;
    //   public static final int LONG_ARM_AUTO_HOOK_BUTTON = 9;
  
    //   // Calculations
    //   public static final double MAX_ARM_EXTENSION = 18500 / 4096D;
    //   public static final double ARM_EXTENSION_DEADZONE = 100 / 4096D; // This is in encoder units
    //   public static final double ARM_EXTEND_POSITION_FACTOR = 1; // It's very difficult to calculate this
  
    //   public static final int MAX_ARM_PIVOT = 70;
    //   public static final int ARM_PIVOT_DEADZONE = 10; // This is in degrees
    //   public static final double ARM_PIVOT_POSITION_FACTOR = 360 * (1 / 100D); // It has a 1 to 100 gear ratio meaning thats 100 rotations per 360 degrees
  
  
    //   /*
    //    * Intake
    //    */
    //   // CAN IDs
    //   public static final int INTAKE_MOTOR = 5;
  
    //   // PCM IDs
    //   public static final int INTAKE_LEFT_DEPLOY = 0;
    //   public static final int INTAKE_LEFT_RETRACT = 1;
    //   public static final int INTAKE_RIGHT_DEPLOY = 2;
    //   public static final int INTAKE_RIGHT_RETRACT = 3;
  
    //   // Input
    //   // The intake start and reverse are handled directly in input manager
    // //public static final int INTAKE_START_BUTTON = 12;
    // //public static final int INTAKE_REVERSE_BUTTON = 11;
    // public static final int INTAKE_HOLD_BUTTON = 1;
    // public static final int INTAKE_TOGGLE_BUTTON = 2;
}
