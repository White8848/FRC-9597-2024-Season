package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class VelocityShootCommand extends Command {

    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Timer m_timer;
    private final double m_shooterVelocitySet = 40.0;
    private final double m_intakeVelocitySet = -30.0;

    public VelocityShootCommand(Intake intake, Shooter shooter) {
        m_intake = intake;
        m_shooter = shooter;
        m_timer = new Timer();

        addRequirements(m_intake, m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setVelocity(m_shooterVelocitySet);
        m_intake.setVelocity(0.0);
        m_timer.reset();
        m_timer.start();

    }

    @Override
    public void execute() {
        var m_shooterVelocity = m_shooter.getUpShooterTalonFX().getVelocity().getValue();
        if (m_timer.get() > 0.5 && m_shooterVelocity > m_shooterVelocitySet * 0.9) {
            m_intake.setVelocity(m_intakeVelocitySet);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setVelocity(0.0);
        m_intake.setVelocity(0.0);

    }

}
