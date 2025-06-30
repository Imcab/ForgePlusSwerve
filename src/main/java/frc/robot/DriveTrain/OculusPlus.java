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
    public Pose2d getPose(){
        return nav.getPose().transformBy(robotToQuest.inverse());
    }

    @AutoNetworkPublisher(key = "questPoseFix")
    public Pose2d getRobotPose(){
        Translation2d fixed = new Translation2d(-getPose().getX(), -getPose().getY());
        return new Pose2d(fixed, getPose().getRotation());
    }

    @AutoNetworkPublisher(key = "questPoseFix2")
    public Pose2d getRobotPose2(){
        Translation2d fixed = new Translation2d(Math.abs(getPose().getX()), Math.abs(getPose().getY()));
        return new Pose2d(fixed, getPose().getRotation());
    }

    public void setPose(Pose2d pose){
        Pose2d questPose = pose.transformBy(robotToQuest);

        nav.setPose(questPose);
    }

    public void resetPose(){
        setPose(new Pose2d());
    }

    public void setPoseSafe(Pose2d fieldPose) {
    // Transforma del centro del robot al headset
    Pose2d questPoseRaw = fieldPose.transformBy(robotToQuest);

    Translation2d trans = questPoseRaw.getTranslation();
    Rotation2d rot = questPoseRaw.getRotation();

    // Clamp negativo a cero (por si el Quest no soporta coordenadas negativas)
    double x = Math.max(0.0, trans.getX());
    double y = Math.max(0.0, trans.getY());

    Pose2d clampedPose = new Pose2d(new Translation2d(x, y), rot);

    nav.setPose(clampedPose);
}


    @NetworkCommand("reset")
    public Command resetOculus(){
        return Commands.runOnce(()-> {setPose(Pose2d.kZero);}, this);
    }

    @NetworkCommand("test")
    public Command test(){
        return Commands.runOnce(()-> {setPose(new Pose2d(3.24, 4.04, new Rotation2d()));}, this);
    }

    @Override
    public void NetworkPeriodic(){
        
    }

    public void update(){
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
