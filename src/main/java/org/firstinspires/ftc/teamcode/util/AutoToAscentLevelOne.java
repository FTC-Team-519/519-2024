package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.events.DriveForward;
import org.firstinspires.ftc.teamcode.util.events.DriveSideways;
import org.firstinspires.ftc.teamcode.util.events.LinearProcessor;

@Autonomous(name="AutoToAscentLevelOne",group="Autonomi")
public class AutoToAscentLevelOne extends OpMode {
    TritonRobot robot;
    LinearProcessor linearProcessor;

    @Override
    public void init() {
        robot = new TritonRobot(hardwareMap);
        linearProcessor = new LinearProcessor(
                new DriveForward(54,robot),
                new DriveSideways(6,robot)
        );
    }
    @Override
    public void loop() {
            linearProcessor.run();
            if(linearProcessor.isDone()){
                telemetry.addLine("Robot Is Finished With Task");
                telemetry.update();
            }
    }
}
