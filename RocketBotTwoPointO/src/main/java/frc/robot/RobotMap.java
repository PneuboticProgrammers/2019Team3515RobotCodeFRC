/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  
  //PWM
  public static final int LeftArmMotorPort = 0;
  public static final int RightArmMotorPort = 1;
  public static final int LeftJawMotorPort = 2;
  public static final int RightJawMotorPort = 3; 
  
  //DIO
  public static final int RotaryEncoderChannelA = 0;
  public static final int RotaryEncoderChannelB = 1;
  public static final int ultraPort = 1;
  public static final int ultraPingPort = 1;

  //CAN
  public static final int LeftMasterMotorPort = 1;
  public static final int RightMasterMotorPort = 0;
  public static final int LeftSlaveMotorPort = 2;
  public static final int RightSlaveMotorPort = 3;
  
  //OI
  public static final int JoyPort = 0;

  //PID Config
    //Arm Subsystem
      public static final double kP = 10.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kF = 0.0;
      public static final double tolerance = 2.0;
    //DriveTrain Subsystem
      public static final double kdP = 10.0;
      public static final double kdI = 0.0;
      public static final double kdD = 0.0;
      public static final double kdF = 0.0;
      public static final double dTtolerance = 2.0;
      public static final double driveRange = 50.0;

  //Speed Constraints
  public static final double maxSpeed = 0.7;
  public static final double deadzone = 0.7;
  public static final double highRocketSetpoint = 5.0;
  public static final double midRocketSetpoint = 2.5;
  public static final double lowRocketSetpoint = 0.0; 
}
