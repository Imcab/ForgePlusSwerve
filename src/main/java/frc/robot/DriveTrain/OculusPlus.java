package frc.robot.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import gg.questnav.questnav.QuestNav;
import lib.ForgePlus.Math.StandardDeviations;

import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.NetworkCommand;

public class OculusPlus extends NetworkSubsystem{

    private final QuestNav nav;
    private final Transform2d robotToQuest;

    public final StandardDeviations dev = new StandardDeviations(0.05, 0.05, 0.035);

    public OculusPlus(Transform2d transform) {
        super("QuestNav/FORGE", false);
        this.nav = new QuestNav();
        this.robotToQuest = transform;
    }

    @AutoNetworkPublisher(key = "questPose")
    public Pose2d getBotPose(){
        return nav.getPose().transformBy(robotToQuest.inverse());
    }

    @AutoNetworkPublisher(key = "questPoseFix")
    public Pose2d fix(){
        Translation2d fixed = new Translation2d(-getBotPose().getX(), -getBotPose().getY());
        return new Pose2d(fixed, getBotPose().getRotation());
    }

    public void setPose(Pose2d pose) {
        Pose2d desired = pose.transformBy(robotToQuest);
        nav.setPose(desired);
    }

    public void resetPose(){
        setPose(new Pose2d());
    }

    @NetworkCommand("reset")
    public Command resetOculus(){
        return Commands.runOnce(()-> {setPose(Pose2d.kZero);}, this);
    }

    @Override
    public void NetworkPeriodic(){
        nav.commandPeriodic();
    }

    @AutoNetworkPublisher(key = "hasPose")
    public boolean hasPose(){
        return nav.isConnected() && nav.isTracking();
    }

    public double timestamp(){
        return nav.getDataTimestamp();
    }

}
