/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  private Encoder rotaryEncoder;
  private Spark leftArmMotor;
  private Spark rightArmMotor;

  public ArmSubsystem() {
    // Insert a subsystem name and PID values here
    
    super("SubsystemName", RobotMap.kP, RobotMap.kI, RobotMap.kD);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    setAbsoluteTolerance(0.2);
    getPIDController().setContinuous(false);
    getPIDController().setName("ArmSubsystem", "PIDSubsystem Controller");

    rotaryEncoder = new Encoder(0, 1, false, EncodingType.k4X);
    rotaryEncoder.setDistancePerPulse(256.0);
    rotaryEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    
    leftArmMotor = new Spark(0);
    leftArmMotor.setInverted(false);
    
    rightArmMotor = new Spark(1);
    rightArmMotor.setInverted(true);

    //Setting the sendableBase and putting it on LiveWindow
    setName("armSubsystemComponents");
    addChild("RotaryEncoder", rotaryEncoder);
    addChild("LeftArmMotor",leftArmMotor);
    addChild("RightArmMotor",rightArmMotor);
    addChild(getPIDController());
    
    LiveWindow.add(Robot.armSubsystem);
    // LiveWindow.addSensor("Arm Subsystem", "Rotary Encoder", rotaryEncoder); //I may have to change this to rotaryEncoder.getDistance()    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return rotaryEncoder.getDistance();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    leftArmMotor.pidWrite(output);
    rightArmMotor.pidWrite(output);
  }
}
