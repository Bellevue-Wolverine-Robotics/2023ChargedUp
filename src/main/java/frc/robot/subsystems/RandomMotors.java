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
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxRelativeEncoder;
import frc.robot.Constants;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;
import java.util.ArrayList; 


public class RandomMotors extends SubsystemBase {
    private int[][] motor_contants = {{}, {}, {}};
    private ArrayList<MotorControllerGroup> motor_cluster = new ArrayList<MotorControllerGroup>();

    public RandomMotors(){

        for(int motorGroupID = 0; motorGroupID < motor_contants.length; motorGroupID++){
            ArrayList<CANSparkMax> motorGroup = new ArrayList<CANSparkMax>();
            for(int motor = 0; motor < motor_contants[motorGroupID].length; motor++){
                motorGroup.add(new CANSparkMax(motor_contants[motorGroupID][motor], MotorType.kBrushless));
                motorGroup.get(motor).restoreFactoryDefaults();
            }
          //  motor_cluster.add(new MotorControllerGroup(motorGroup.get(0))); 
            motor_cluster.add(new MotorControllerGroup((MotorController[]) (motorGroup.toArray()))); 
            //MotorControllerGroup
        }
    }
    public void setSpeed(int clusterID, double speed){
        motor_cluster.get(clusterID).set(speed);
    }
    public void setInverted(int clusterID, boolean inverted){
        motor_cluster.get(clusterID).setInverted(inverted);
    }


}
