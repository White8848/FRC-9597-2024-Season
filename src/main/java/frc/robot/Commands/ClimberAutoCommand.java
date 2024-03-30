package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class ClimberAutoCommand extends Command {

    private final LeftClimber m_LeftClimber;
    private final RightClimber m_RightClimber;
    private double m_startPositionDifference = 0.0;
    private double m_currentPositionDifference = 0.0;

    enum ClimberState {
        UP, DOWN;
    }

    ClimberState m_state;

    PIDController m_pidController = new PIDController(5, 0.0, 0.0);

    public ClimberAutoCommand(LeftClimber leftClimber, RightClimber rightClimber, String state) {
        m_LeftClimber = leftClimber;
        m_RightClimber = rightClimber;

        if (state.equals("UP")) {
            m_state = ClimberState.UP;
        } else {
            m_state = ClimberState.DOWN;
        }

        addRequirements(m_LeftClimber, m_RightClimber);
    }

    @Override
    public void initialize() {
        m_LeftClimber.positionTarget = m_LeftClimber.getTalonFX().getPosition().getValueAsDouble();
        m_RightClimber.positionTarget = m_RightClimber.getTalonFX().getPosition().getValueAsDouble();
        m_startPositionDifference = m_LeftClimber.positionTarget - m_RightClimber.positionTarget;
    }

    @Override
    public void execute() {
        switch (m_state) {
            case UP:
                m_LeftClimber.setVelocity(50.0);
                m_LeftClimber.positionTarget = m_LeftClimber.getTalonFX().getPosition().getValueAsDouble();
                m_RightClimber.positionTarget = m_LeftClimber.getTalonFX().getPosition().getValueAsDouble()
                        - m_startPositionDifference;
                m_RightClimber.setPosition(m_RightClimber.positionTarget);
                break;
            case DOWN:
                m_LeftClimber.setVelocity(-50.0);
                m_LeftClimber.positionTarget = m_LeftClimber.getTalonFX().getPosition().getValueAsDouble();
                m_RightClimber.positionTarget = m_LeftClimber.getTalonFX().getPosition().getValueAsDouble()
                        - m_startPositionDifference;
                m_RightClimber.setPosition(m_RightClimber.positionTarget);

                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_LeftClimber.setVelocity(0.0);
        m_RightClimber.setVelocity(0.0);
        m_LeftClimber.setPosition(m_LeftClimber.positionTarget);
        m_RightClimber.setPosition(m_RightClimber.positionTarget);
    }

}
