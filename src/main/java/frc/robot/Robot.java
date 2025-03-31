// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Controls.RobotContainer;
import frc.robot.Subsystem.Drivetrain;

public class Robot extends TimedRobot {
  private RobotContainer robotcontainer;
  private Command autocommand;
  public Drivetrain drivetrain;

  public Robot(){
    drivetrain = new Drivetrain();

    initialize();
  }
  
  public void initialize(){
    drivetrain.drive(0, 0, false);
  }

  @Override
  public void robotInit(){
    robotcontainer = new RobotContainer();

  }

  @Override
  public void teleopPeriodic(){
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit(){
    if(autocommand != null){
      autocommand.cancel();
    }
  }

  @Override
  public void autonomousInit(){
    autocommand = robotcontainer.getAutoCommand();

    if(autocommand != null){
      autocommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic(){
    CommandScheduler.getInstance().run();
  }
}
