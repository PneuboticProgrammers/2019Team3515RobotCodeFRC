/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDriveCommand;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX leftMasterMotor;
  private WPI_TalonSRX rightMasterMotor;
  private WPI_VictorSPX leftSlaveMotor;
  private WPI_VictorSPX rightSlaveMotor;
  private DifferentialDrive drive;
  private Ultrasonic ultraRange;

  public DriveTrainSubsystem() {
    //Setting PID Constraints
    super("DriveTrainSubsystem", RobotMap.kdP, RobotMap.kdI, RobotMap.kdD);
    setAbsoluteTolerance(RobotMap.dTtolerance);

    getPIDController().setContinuous(false);
    getPIDController().setName("DriveTrainSubsystem", "PIDSubsystem Controller");

    //I will use Ultrasonic for now, but hoepfully we can tune PID for the MAG SRX encoders later
    ultraRange = new Ultrasonic(RobotMap.ultraPort, RobotMap.ultraPingPort); //DIO channel, Digital eco output channel, both DIO ports I think
    ultraRange.setAutomaticMode(true);

    leftMasterMotor = new WPI_TalonSRX(0);
    rightMasterMotor = new WPI_TalonSRX(1);
    leftSlaveMotor = new WPI_VictorSPX(2);
    rightSlaveMotor = new WPI_VictorSPX(3);
    
    drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
    leftSlaveMotor.follow(leftMasterMotor);
    rightSlaveMotor.follow(rightMasterMotor);

    setName("DriveTrain Subsystem Components");
    addChild("Left Master", leftMasterMotor);
    addChild("Right Master", rightMasterMotor);
    addChild("Left Slave", leftSlaveMotor);
    addChild("Right Slave", rightSlaveMotor);
    addChild("Ultrasonic", ultraRange);
    addChild(getPIDController());

    LiveWindow.add(this);
  }

  public void setDrive(double power, double angle){
    //setting the max Speed of the drive
    if(power > RobotMap.maxSpeed) power = RobotMap.maxSpeed;
    else if(power < -RobotMap.maxSpeed) power = -RobotMap.maxSpeed;

    //setting the deadzone of the controller
    if(Math.abs(power) < RobotMap.deadzone) power = 0;

    if((angle > -0.1) && (angle < 0.1)){
      leftMasterMotor.set(power);
      rightMasterMotor.setInverted(true);
      rightMasterMotor.set(power);
    }
    else{
      rightMasterMotor.setInverted(false);
      drive.arcadeDrive(power, angle);
    }
    
    SmartDashboard.putNumber("Ultrasonic Range", returnPIDInput());
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ArcadeDriveCommand());
  }

  @Override
  protected double returnPIDInput() {
    return ultraRange.getRangeInches();
  }

  @Override
  public void usePIDOutput(double output) {
    leftMasterMotor.pidWrite(output);

    rightMasterMotor.setInverted(true);
    rightMasterMotor.pidWrite(output);

    SmartDashboard.putNumber("PID Ultrasonic Range", returnPIDInput());
  }
}
