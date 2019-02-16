/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ManualJawCommand;

/**
 * Add your docs here.
 */
public class JawSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Spark leftJawMotor;
  private Spark rightJawMotor;

  public JawSubsystem(){
    leftJawMotor = new Spark(2);
    //addChild("LeftJawMotor",leftJawMotor);
    leftJawMotor.setInverted(false);
    
    rightJawMotor = new Spark(3);
    //addChild("RightJawMotor",rightJawMotor);
    rightJawMotor.setInverted(true);
  }

  public void setSpeed(double speed){
    leftJawMotor.set(speed);
    rightJawMotor.set(-speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualJawCommand());
  }
}
