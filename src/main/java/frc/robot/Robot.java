/**
 * Based on WPILib Command Robot Template
 */

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Threads;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.wpilog.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", "2026RobotCode");

        // set up data receivers for advantagekit
        switch (Constants.CURRENT_MODE) {
            case REAL: // log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM: // log to NetworkTables
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY: // set up replay source
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

        robotContainer.updateDriverInfo();
    }

    @Override
    public void disabledInit() {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM) {
            robotContainer.resetSimulation();
        }
    }

    @Override
    public void disabledPeriodic() {
        robotContainer.updateSelectedAutoDrawing();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the auto command
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) { // stops auto when teleop starts
            autonomousCommand.cancel();
        }

        robotContainer.resetSimulation();
    }

    @Override
    public void teleopPeriodic() {}

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
        robotContainer.updateSimulation();
    }
}