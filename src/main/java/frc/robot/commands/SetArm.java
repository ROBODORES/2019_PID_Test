/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;

public class SetArm extends Command {
  public SetArm() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_arm.setSetpoint(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.m_oi.sideStick.getRawButton(2)) {
      Robot.m_arm.setSetpoint(5.0);
    } else if (Robot.m_oi.sideStick.getRawButton(6)) {
      Robot.m_arm.setSetpoint(60.0);
    } else if (Robot.m_oi.sideStick.getRawButton(4)) {
      Robot.m_arm.setSetpoint(104.0);
    }
    Robot.m_arm.enable();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_arm.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
