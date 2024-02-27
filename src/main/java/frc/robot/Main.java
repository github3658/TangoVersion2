// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// We don't use this file for anything. This is just to set up the framework of the robot code.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  	private Main() {}

  	public static void main(String... args) {
    	RobotBase.startRobot(Robot::new);
  	}
}
