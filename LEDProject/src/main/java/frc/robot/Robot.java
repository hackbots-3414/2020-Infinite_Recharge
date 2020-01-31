/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  private Joystick joy = new Joystick(0);
  private int m_lastValue=0;
  private boolean m_isDecreasing=false;

  @Override
  public void robotInit() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(150);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void robotPeriodic() {
    // Fill the buffer with a rainbow
    if (joy.getRawButtonPressed(10)) {
    rainbow();
    } else if (joy.getRawButtonPressed(6)) {
solidColor();
    } else if (joy.getRawButtonPressed(8)) {
      colorPulse();
    }

    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }

  private void solidColor() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values for purple
      m_ledBuffer.setHSV(i, 278, 100, 100);
    }
 
   m_led.setData(m_ledBuffer);
  }
private void colorPulse() {
  if (m_isDecreasing) {
    if (m_lastValue==0){
      m_isDecreasing=false;
    } else {
      m_lastValue -= 1;
    }
  } else {
    if (m_lastValue==100){
      m_isDecreasing=true;
    } else {
      m_lastValue += 1;
    }
  }
  
  for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    // Sets the specified LED to the HSV values for dark purple
    m_ledBuffer.setHSV(i, 278, 100, m_lastValue);
  }
}
  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
}
