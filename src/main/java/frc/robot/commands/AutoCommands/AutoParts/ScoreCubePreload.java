// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoParts;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMode;
import frc.robot.commands.ConfirmScore;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ReturnFromScoring;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RunIntakeAtSpeed;
import frc.robot.subsystems.*;

public class ScoreCubePreload extends CommandBase {

    private Elevator elevator;
    private Wrist wrist;
    private Intake intake;

    public ScoreCubePreload(Elevator elevator, Wrist wrist, Intake intake) {

        // Use addRequirements() here to declare subsystem dependencies.
        this.elevator = elevator;
        this.wrist = wrist;
        this.intake = intake;

        addRequirements(wrist, elevator, intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

      new SequentialCommandGroup(

      new InstantCommand(() -> RobotMode.SetMode(RobotMode.ModeOptions.CUBE)),
      new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.STOW)).withTimeout(1.0),
      new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.HIGH)).withTimeout(4.0),
      new ReverseIntake(intake).withTimeout(1.0),
      new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.STOW)).withTimeout(3.0)

    );
        
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