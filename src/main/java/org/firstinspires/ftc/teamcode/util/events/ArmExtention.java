package org.firstinspires.ftc.teamcode.util.events;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.TritonRobot;

// Uses encoders
public class ArmExtention implements Event {
    private TritonRobot robot;
    private double desiredPosition;
    private double startingPosition;

    public ArmExtention(double position, TritonRobot robot) {
        // init robot
        this.robot = robot;
        desiredPosition = position;
    }

    public void init() {
        startingPosition = robot.getCurrentLeftSpoolPosition();
        this.robot.setSpoolMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.setSpoolTargetPosition((int)desiredPosition);
    }

    // A simplified boolean to decide whether we are extending or retracting
    public boolean extending() {
        return (desiredPosition<=startingPosition);
    }

    //    @Override
    public void run() {
        if(extending()) {
            this.robot.setSpoolPower(0.5);
        }
        else {
            this.robot.setSpoolPower(-0.5);
        }
    }

    //@Override
    public void shutdown() {
        robot.setSpoolPower(0);
    }

    //    @Override
    public boolean isDone() {
        if(extending()) {
            return robot.atSpoolPositionHigh();
        }
        else {
            return robot.atSpoolPositionLow();
        }
    }
}
