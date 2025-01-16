package org.firstinspires.ftc.teamcode.util.events;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.TritonRobot;
//CLOCKWISE IS NEGATIVE IMU, positive boolean is clockwise, negative is counterclockwise
public class Rotate implements Event {
    private double targetAngle;
    private double startingAngle;
    private boolean direction;
    private TritonRobot robot;

    public Rotate(double angle, boolean direction, TritonRobot robot) {
        this.targetAngle = angle;
        startingAngle = robot.getYaw();
        this.direction = direction;
        // init robot
        this.robot = robot;

    }

    public void init() {
        this.robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void run() {

        if(!direction) {
            robot.rotateCounterClockwise(0.5);
        }
        else {
            robot.rotateClockwise(0.5);
        }
    }

    public void shutdown() {
        this.robot.setAllDrivePower(0.0d);
        robot.resetYaw();
        this.robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isDone() {
        if(!direction && targetAngle>0){
            return robot.getYaw() <= targetAngle && robot.getYaw()>0;
        } else if(direction && targetAngle<0) {
            return robot.getYaw() >=targetAngle && robot.getYaw()<0;
        } else if(direction && targetAngle>0){
            return robot.getYaw() >= targetAngle;
        } else {
            return robot.getYaw() <= targetAngle;
        }
    }
}
