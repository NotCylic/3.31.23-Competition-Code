// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

public class AutonomousC extends CommandBase {
  DriveSubsystem autoDrive;
  Intake intake;
  Timer timer;
  public boolean finish = false;
  /** Creates a new AutonomousC. */
  public AutonomousC(DriveSubsystem AD, Intake IT) {
    autoDrive = AD;
    addRequirements(autoDrive);
    intake = IT;
    addRequirements(intake);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    if(timer.get() < 1) {
      intake.dropObject(-.3);
    }
    if(timer.get() > 1 && timer.get() < 2) {
      intake.stop();
      autoDrive.drive(0.3,0,0,false,false);
    }
    if(timer.get() > 2) {
     autoDrive.drive(0,0,0,false,false);
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
