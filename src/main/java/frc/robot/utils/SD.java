// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */

public class SD {

  public static void sd2(String name, double number) {
    double temp = 100;// Math.pow(10, 2);
    double temp1 = Math.round(number * temp);
    SmartDashboard.putNumber(name, temp1 / temp);
  }

  public static void sd1(String name, double number) {
    double temp = 10;// Math.pow(10, 1);
    double temp1 = Math.round(number * temp);
    SmartDashboard.putNumber(name, temp1 / temp);
  }

  public static void sd(String name, double number) {
    SmartDashboard.putNumber(name, Math.round(number));
  }

}
