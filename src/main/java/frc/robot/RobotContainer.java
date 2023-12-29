// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;  
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Loop.DriveLoop;
import frc.robot.Loop.DriveLoop.DriveStates;

public class RobotContainer {
  private DriveLoop drive;

  public RobotContainer() {
     drive = DriveLoop.getInstance();
    
  }

  public void initTeleop(){
    System.out.println("TELEOP *******************************");
     drive.setDriveState(DriveStates.OPEN_LOOP);
  }

  public void initDisabled(){
     System.out.println("DISABLED *******************************");
     drive.setDriveState(DriveStates.DISABLED);
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
