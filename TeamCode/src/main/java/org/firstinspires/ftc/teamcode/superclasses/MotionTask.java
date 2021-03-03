package org.firstinspires.ftc.teamcode.superclasses;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;

public class MotionTask extends Pose2D {


    public Runnable actionOnConpletion = () -> {
    };

    public MotionTask(Pose2D pose) {
        super(pose.x, pose.y, pose.heading);
    }

    public MotionTask(double x, double y, double heading, Runnable actionOncompletion) {
        super(x, y, heading);
        this.actionOnConpletion = actionOncompletion;
    }

    public MotionTask(Vector2D vector, double heading, Runnable actionOncompletion) {
        super(vector.x, vector.y, heading);
        this.actionOnConpletion = actionOncompletion;
    }

    public MotionTask(Vector2D vector, double heading) {
        super(vector.x, vector.y, heading);
    }

    public MotionTask(Pose2D pose, Runnable actionOncompletion) {
        super(pose.x, pose.y, pose.heading);
        this.actionOnConpletion = actionOncompletion;
    }

    public MotionTask(double x, double y, double heading) {
        super(x, y, heading);
    }

    public MotionTask(double x, double y, Runnable actionOncompletion) {
        super(x, y, Double.NaN);
        this.actionOnConpletion = actionOncompletion;
    }

    public MotionTask(double x, double y) {
        super(x, y, Double.NaN);
    }

    @Override
    public MotionTask clone() {
        return new MotionTask(this.x,this.y,this.heading,this.actionOnConpletion);
    }


}
