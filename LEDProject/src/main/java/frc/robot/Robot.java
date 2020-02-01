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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class Robot extends TimedRobot {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  private int m_chaseLightIndex=0;
  private Joystick joy = new Joystick(0);
  private int m_lastValue=0;
  private boolean m_isDecreasing=false;
    /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

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

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void robotPeriodic() {
//     // Fill the buffer with a rainbow
//     if (joy.getRawButtonPressed(10)) {
//       rainbow();
//     } else if (joy.getRawButtonPressed(6)) {
// solidColor();
//     } else if (joy.getRawButtonPressed(8)) {
//       colorPulse();
//     }
  //rainbow();

    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
      solidColorBlue();
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      solidColorRed();
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      solidColorGreen();
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      solidColorYellow();
    } else {
      colorString = "Unknown";
    }

    // Set the LEDs
    m_led.setData(m_ledBuffer);
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  private void solidColorYellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, 43, 255, 255 );
 }
  }

  private void solidColorGreen() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, 61, 255, 255);
 }
  }

  private void solidColorRed() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, 0, 255, 255);
 }
  }

  private void solidColorBlue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
         m_ledBuffer.setHSV(i, 120, 255, 255);
    }
  }
  private static final int MAX_NO_OF_LEDS = 150;

  private void lightChaseV2(int currentIndex) {
    boolean forwadDirection = true;
    if(currentIndex >= MAX_NO_OF_LEDS) {

    }
  }
  private void lightChase() {
    //Turn the current light off
    m_ledBuffer.setHSV(m_chaseLightIndex, 270, 100, 0);
    //Now increase of decrase the index for the next light to turn on 
    if (!m_isDecreasing && (m_chaseLightIndex < m_ledBuffer.getLength()-1)) { 
      m_chaseLightIndex ++;
    } else if (m_isDecreasing && m_chaseLightIndex > 0){
      m_chaseLightIndex --;
    }
    m_ledBuffer.setHSV(m_chaseLightIndex, 200, 100, 100);
    if (m_chaseLightIndex==(m_ledBuffer.getLength() -1)) { m_isDecreasing=true;}
    if (m_chaseLightIndex==0) { m_isDecreasing=false;}
  }

  public void solidColor(String coloString){
    m_ledBuffer.setHSV(1, 270, 100, 100);

  }
  private void solidColor() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values for purple
      m_ledBuffer.setHSV(i, 270, 100, 100);
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
    m_ledBuffer.setHSV(i, 200, 100, m_lastValue);
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