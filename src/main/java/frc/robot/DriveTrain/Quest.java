package frc.robot.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import lib.ForgePlus.Math.StandardDeviations;

public class Quest extends SubsystemBase{

    private final QuestNav nav;
    private final Transform2d robotToQuest;

    public final StandardDeviations dev = new StandardDeviations(0.02, 0.02, 0.035);

    public Quest(double x, double y, double rotation) {
        this.nav = new QuestNav();
        this.robotToQuest = new Transform2d(x, y, Rotation2d.fromDegrees(rotation));
    }

    public Pose2d getBotPose(){
        return nav.getPose().transformBy(robotToQuest.inverse());
    }

    public void setPose(Pose2d pose) {
        Pose2d desired = pose.transformBy(robotToQuest);
        nav.setPose(desired);
    }

    @Override
    public void periodic(){
        nav.commandPeriodic();
    }

    public QuestNav getQuestNav() {
        return nav;
    }


}
