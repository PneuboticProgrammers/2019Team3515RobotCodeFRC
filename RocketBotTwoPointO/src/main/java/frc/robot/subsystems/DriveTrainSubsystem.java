/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDriveCommand;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX leftMasterMotor;
  private WPI_TalonSRX rightMasterMotor;
  private WPI_VictorSPX leftSlaveMotor;
  private WPI_VictorSPX rightSlaveMotor;
  private DifferentialDrive drive;

  public DriveTrainSubsystem() {
    leftMasterMotor = new WPI_TalonSRX(0);
    rightMasterMotor = new WPI_TalonSRX(1);
    leftSlaveMotor = new WPI_VictorSPX(2);
    rightSlaveMotor = new WPI_VictorSPX(3);
    
    drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
    leftSlaveMotor.follow(leftMasterMotor);
    rightSlaveMotor.follow(rightMasterMotor);

    // initSendable(leftMasterBuilder);
    // leftMasterBuilder
    // LiveWindow.addChild("leftMasterMotor", leftMasterMotor);
  }

  public void setDrive(double power, double angle){
    //setting the max Speed of the drive
    if(power > RobotMap.maxSpeed) power = RobotMap.maxSpeed;
    else if(power < -RobotMap.maxSpeed) power = -RobotMap.maxSpeed;

    //setting the deadzone of the controller
    if(Math.abs(power) < RobotMap.deadzone) power = 0;

    if((angle > -0.1) && (angle < 0.1)){
      leftMasterMotor.set(power);
    }
    else{
      drive.arcadeDrive(power, angle);
    }
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ArcadeDriveCommand());
  }
}
