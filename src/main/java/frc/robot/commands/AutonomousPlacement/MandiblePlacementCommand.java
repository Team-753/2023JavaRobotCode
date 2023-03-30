package frc.robot.commands.AutonomousPlacement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Mandible;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;

public class MandiblePlacementCommand extends CommandBase {
    Mandible mandible;
    boolean finished = false;
    Timer timer = new Timer();

    public MandiblePlacementCommand(Mandible kMandible) {
        mandible = kMandible;
        addRequirements(mandible);
    }

    @Override
    public void initialize() {
        if (mandible.open) { // it's a cube, we need to shoot it
            timer.restart();
            mandible.outtakeWheels();
        }
        else { // it's a cone, we just need to open the mandible
            mandible.setOpen(true);
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        mandible.passiveIntake();
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return finished || timer.hasElapsed(Config.MandibleConstants.defaultOuttakeTime);
    }
}
