/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.HighArmCommand;
import frc.robot.commands.LowArmCommand;
import frc.robot.commands.MidArmCommand;
import frc.robot.commands.PrepareHighShot;
import frc.robot.commands.PrepareLowShot;
import frc.robot.commands.PrepareMidShot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // controller = new Joystick(0);
  
  // jawOutputButton = new JoystickButton(controller, 2);
  // jawOutputButton.whenPressed(new JawOutputCommand());
  // jawIntakeButton = new JoystickButton(controller, 1);
  // jawIntakeButton.whenPressed(new JawIntakeCommand());
  public Joystick joy;

  JoystickButton lowRocketButton;
  JoystickButton midRocketButton;
  JoystickButton highRocketButton;

  public OI(){
    joy = new Joystick(RobotMap.JoyPort);
    lowRocketButton = new JoystickButton(joy, 1);
    midRocketButton = new JoystickButton(joy, 2);
    highRocketButton = new JoystickButton(joy, 3);

    lowRocketButton.whileHeld(new LowArmCommand());
    midRocketButton.whileHeld(new MidArmCommand());
    highRocketButton.whileHeld(new HighArmCommand());

    //SmartDashboard Buttons
    SmartDashboard.putData("PrepareLowShot", new PrepareLowShot());
    SmartDashboard.putData("PrepareMidShot", new PrepareMidShot());
    SmartDashboard.putData("PrepareHighShot", new PrepareHighShot());

  }
  
  

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
