package org.firstinspires.ftc.teamcode.util.events;

import org.firstinspires.ftc.teamcode.util.TritonRobot;
import org.firstinspires.ftc.teamcode.util.events.processors.ParallelProcessor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class ArmPositionForBucket implements Event{
    TritonRobot robot;
    ParallelProcessor parallelProcessor;

    public ArmPositionForBucket(TritonRobot robot) {
        this.robot = robot;
    }

    public void init() {
         parallelProcessor = new ParallelProcessor(
                new ArmRotation(17, this.robot),
                new ArmExtension(-3157, 1, this.robot)
        );
         parallelProcessor.init();
    }

    public void run() {
        parallelProcessor.run();
        if(parallelProcessor.isDone()) {
            parallelProcessor.shutdown();
        }
    }

    public void shutdown() { parallelProcessor.shutdown(); }

    public boolean isDone() { return parallelProcessor.isDone(); }

}
