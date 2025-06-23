package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.RobotState.RobotLifeCycle;

public final class RobotState extends SubsystemBase implements RobotLifeCycle{

    private static RobotState instance;

    private RobotState() {
   
    }

    public static synchronized RobotState getInstance(){

        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    @Override
    public void periodic(){
        NTPublisher.updateAllSendables();

        NTPublisher.publish(NTPublisher.ROBOT, "Match/Time", DriverStation.getMatchTime());

    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}


}
