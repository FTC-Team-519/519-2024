package org.firstinspires.ftc.teamcode.util.events;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.TritonRobot;

// Uses encoders
public class ArmExtensionDown implements Event {
    private TritonRobot robot;
    private double desiredPosition;
    private double startingPosition;

    public ArmExtensionDown(double position, TritonRobot robot) {
        // init robot
        this.robot = robot;
        desiredPosition = position;
        startingPosition = robot.getCurrentLeftSpoolPosition();
    }

    public void init() {
        this.robot.setSpoolMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.setSpoolTargetPosition((int)desiredPosition);
    }

    //    @Override
    public void run() {
        this.robot.setSpoolPower(-0.5);
    }

    //@Override
    public void shutdown() {
        robot.setSpoolPower(0);
    }

    //    @Override
    public boolean isDone() {
        return robot.atSpoolPositionLow();
    }
}
