/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LedController sends PWM signals to the REV Blinkin to change the color of LEDs
 */
public class LEDSubsystem extends SubsystemBase {

  private static final int LED_CONTROLLER_ID = 0; // move to RobotMap.java

  /** LED controller acts as a Spark motor controller for it's inputs for changing patterns. */
  private Spark LED_controller;
  private int CurrentColor = 0;
  private int CurrentPattern = 0;
  
  // all available patterns are at http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14

  /**
   * Spark value for color of the LED strip when there is no other command.
   * <p>
   * 43 Fixed Pallete Pattern Breath, Blue
   */
  public static final double IDLE = -0.15;
  double[][] ColorPatternArray = {{-0.11, 0.99, -0.05, -0.09, 0.15, 0.35}, 
  {-0.31, -0.27, 0.99, -0.29, 0.01, 0.21},
  {-0.25, -0.19, -0.21, -0.23, 0.07, 0.27},
  {-0.17, -0.13, 0.99, -0.15, 0.11, 0.31}, 
  {-0.85, 0.99, -0.81, -0.83, 0.13, 0.33}, 
  {0.61, 0.95, 0.93, 0.87, 0.99, 0.99}};

  // check with drivers what indicators are needed
  


  public LEDSubsystem() {
    LED_controller = new Spark(LED_CONTROLLER_ID);
  }

  public void updatePattern(int Pattern){
    CurrentPattern = Pattern;
    Update();
  }
  public void updateColor(int Color){
    CurrentColor = Color;
    Update();
  }
  public void Update(){
    setChannel(ColorPatternArray[CurrentPattern][CurrentColor]);
  }

  /**
   * Change the pattern of the LED Controller by using preset channel number in {@link LedControllerSubsystem}.
   * @param channel id as a spark value
   */
   public void setChannel(double channel) {
    LED_controller.set(channel);
  }

  /**
   * Gets the spark value that the LED Controller is currently using
   * @return spark value of current pattern
   */
  public double getChannel() {
    return LED_controller.get();
  }

  @Override
	public void periodic() {}
}