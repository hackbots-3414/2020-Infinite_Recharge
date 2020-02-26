/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class OI {
  static XboxController pad = new XboxController(0);
  static XboxController pad2 = new XboxController(1);
  private static final int leftYAxis = 1;
  private static final int leftXAxis = 2;
  private static final int rightYAxis = 3;
  private static final int rightXAxis = 4;

  //BUTTON MAPPINGS
  public static final int Y_BTN_SHOOTSEQUENCE = 4;
  public static final int X_BTN_ALIGNANDSHOOT = 3;
  public static final int A_BTN_LIMELIGHTALIGN = 1;
  public static final int LB_BTN_INTAKE = 5;
  public static final int RB_BTN_HOOK_POSITIVE = 6;
  public static final int CENTERLEFT_BTN_HOOK_NEGATIVE = 7;
  public static final int B_BTN_PULLY = 2;


  public static XboxController getDrivePad() {
    return pad;
  }

  public static XboxController getOperatorPad() {
    return pad2;
  }

}