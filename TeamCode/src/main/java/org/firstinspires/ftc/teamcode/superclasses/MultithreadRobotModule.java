package org.firstinspires.ftc.teamcode.superclasses;

@SuppressWarnings("EmptyMethod")
public abstract class MultithreadRobotModule extends RobotModule {

    @Override
    public final void updateAll() {
        updateControlHub();
        updateExpansionHub();
        updateOther();
    }

    public void updateControlHub() {
    }

    public void updateExpansionHub() {
    }

    public void updateOther() {
    }
}
