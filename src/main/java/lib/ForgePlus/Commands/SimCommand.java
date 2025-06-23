package lib.ForgePlus.Commands;

import edu.wpi.first.wpilibj.RobotBase;

public interface SimCommand{

    public default boolean isInSim(){
        return RobotBase.isSimulation();
    }

    public default boolean isInReal(){
        return RobotBase.isReal();
    }

    public void simInit();

    public void realInit();

    public default void initReality() {
        if (isInSim()) {
            simInit();
        } else if (isInReal()) {
            realInit();
        }
    }

    public void simPeriodic();
    public void realPeriodic();

    public default void periodicReality() {
        if (isInSim()) {
            simPeriodic();
        } else if (isInReal()) {
            realPeriodic();
        }
    }

    public void simExit();
    public void realExit();

    public default void exitReality() {
        if (isInSim()) {
            simExit();
        } else if (isInReal()) {
            realExit();
        }
    }

}
