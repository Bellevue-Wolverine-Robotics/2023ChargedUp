// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class PneumaticsConstants {
    // to be changed
    public static final int INTAKE_FORWARD_CHANNEL = 0;
    public static final int INTAKE_REVERSE_CHANNEL = 1;
  }

  public static class PhysicalConstants {
    // to be changed
    public static final double WHEEL_CIRCUMFERENCE_METERS = 1;
  }

  public static class ButtonConstants {
    public static final int INTAKE_GRAB_BUTTON = 4;
    public static final int INTAKE_RELEASE_BUTTON = 5;
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
