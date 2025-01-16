package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.events.*;

@Autonomous(name="ArmTest",group="Autonomi")
public class ArmTest extends OpMode {
    TritonRobot robot;
    LinearProcessor linearProcessor;

    @Override
    public void init() {
        robot = new TritonRobot(hardwareMap);
        linearProcessor = new LinearProcessor(
                new ParallelProcessor(
                        new ArmPositionForBucket(robot),
                        new DriveForward(12, robot)
                ),
                new ArmExtension(0, robot)
                //new ArmExtension(-3571, robot),
                //new ArmExtensionDown(0, robot)
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
