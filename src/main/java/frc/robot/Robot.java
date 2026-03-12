package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import edu.wpi.first.wpilibj.Threads;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.wpilog.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", "2026RobotCode");

        switch (Constants.CURRENT_MODE) { // set up data receivers
            case REAL: // running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM: // running a physics simulator, log to NetworkTables
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY: // replaying a log, set up replay source
                setUseTiming(false); 
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99); // switch thread to high priority to improve loop timing
        CommandScheduler.getInstance().run();
        Threads.setCurrentThreadPriority(false, 10); // return to normal thread priority
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedules the autonomous command
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }

        robotContainer.resetSimulation();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) { // stops autonomous when teleop starts
            autonomousCommand.cancel();
        }

        robotContainer.resetSimulation();
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.teleopPeriodic();

        Logger.recordOutput("fuck1", DriveConstants.ALLOWED_LINEAR_SPEED.in(MetersPerSecond));
        Logger.recordOutput("fuck2", DriveConstants.ALLOWED_ANGULAR_SPEED.in(RadiansPerSecond));
        Logger.recordOutput("fuck3", DriveConstants.ALLOWED_LINEAR_ACCEL.in(MetersPerSecondPerSecond));
        Logger.recordOutput("fuck4", DriveConstants.ALLOWED_ANGULAR_ACCEL.in(RadiansPerSecondPerSecond));
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll(); // cancels all running commands when test starts
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        robotContainer.resetSimulation();
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.simulationPeriodic();
    }
}