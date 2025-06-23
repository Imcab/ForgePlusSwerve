package lib.ForgePlus.RobotState;

public interface RobotLifeCycle {

    default void teleopInit() {}
    default void teleopPeriodic() {}
    default void teleopExit() {}
    default void autonomousInit() {}
    default void autonomousPeriodic() {}
    default void autonomousExit() {}
    default void disabledPeriodic() {}
    default void disabledExit() {}
    default void disabledInit() {}


}




