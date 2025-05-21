// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Controls.RobotContainer;

public class Robot extends TimedRobot {
  private RobotContainer robotcontainer;
  @Override
  public void robotInit(){
    robotcontainer = new RobotContainer();

  }

  @Override
  public void teleopPeriodic(){
    CommandScheduler.getInstance().run();
  }

}
