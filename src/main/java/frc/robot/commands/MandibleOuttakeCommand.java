package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Mandible;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;

public class MandibleOuttakeCommand extends CommandBase {
        double timerThreshold;
        Timer timer = new Timer();
        Mandible mandible;

        public MandibleOuttakeCommand(Mandible kMandible) {
            mandible = kMandible;
            timerThreshold = Config.MandibleConstants.defaultOuttakeTime;
            addRequirements(mandible);
        }

        public MandibleOuttakeCommand(Mandible kMandible, double timeToSpin) {
            mandible = kMandible;
            timerThreshold = timeToSpin;
            addRequirements(mandible);
        }

        @Override
        public void initialize() {
            mandible.outtakeWheels();
            timer.restart();
        }

        @Override
        public void end(boolean interrupted) {
            mandible.passiveIntake();
            timer.stop();
        }

        @Override
        public boolean isFinished() {
            return timer.hasElapsed(timerThreshold);
        }
    }