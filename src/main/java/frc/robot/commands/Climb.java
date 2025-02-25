// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  ClimbSubsystem m_ClimbSubsystem;

  /** Creates a new Climb. */
  public Climb(ClimbSubsystem parm_climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimbSubsystem = parm_climb;
    addRequirements(m_ClimbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimbSubsystem.Climb();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
