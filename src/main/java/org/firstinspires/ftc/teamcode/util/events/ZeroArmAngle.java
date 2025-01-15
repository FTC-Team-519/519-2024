package org.firstinspires.ftc.teamcode.util.events;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.util.TritonRobot;

public class ZeroArmAngle implements Event{

    TritonRobot robot;

    public ZeroArmAngle(TritonRobot robot){
        this.robot = robot;
    }

    public void init() {}

    @Override
    public void run() {

        if (!this.robot.getTouchFront().isPressed()) {
            robot.setArmRotatePower(-0.1);
        }else{
            robot.setArmRotatePower(0.0);
        }
    }

    public void shutdown() {
        robot.setArmRotatePower(0.0);
    }

    @Override
    public boolean isDone() {
        return this.robot.getTouchFront().isPressed();
    }
}
