/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//ASK MATTHEW INCASE ANYTHING DOESN'T WORK
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
/*
 *
 * This OpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The OpMode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name = "Sensor: My MR Color", group = "Sensor")
public class SensorMRColor extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftDrive = null;
  private DcMotor rightDrive = null;
  ColorRangeSensor colorSensor;    // Hardware Device Object



  public void runOpMode() {
    telemetry.addData("Status", "Initialized");

    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftDrive = hardwareMap.get(DcMotor.class, "left");
    rightDrive = hardwareMap.get(DcMotor.class, "right");

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    leftDrive.setDirection(DcMotor.Direction.REVERSE);
    rightDrive.setDirection(DcMotor.Direction.FORWARD);

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean xPrevState = false;
    boolean xCurrState = false;
    boolean yPrevState = false;
    boolean yCurrState = false;
    // bLedOn represents the state of the LED.
    boolean bLedOn = true;
    boolean sensingColor = true;
    // get a reference to our ColorSensor object.
    colorSensor = hardwareMap.get(ColorRangeSensor.class, "sensor_color");

    // Set the LED in the beginning
    colorSensor.enableLed(bLedOn);

    // wait for the start button to be pressed.
    waitForStart();

    // while the OpMode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      double leftPower;
      double rightPower;
      leftPower = 1;
      rightPower = 1;
      // Show the elapsed game time and wheel power.
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
      // check the status of the x button on either gamepad.
      xCurrState = gamepad1.x;
      yCurrState = gamepad1.y;
      // check for button state transitions.
      if (xCurrState && (xCurrState != xPrevState)) {

        // button is transitioning to a pressed state. So Toggle LED
        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);
      }
      if (yCurrState && (yCurrState != yPrevState)) {
        sensingColor = !sensingColor;
      }
      // update previous state variable.
      xPrevState = xCurrState;
      yPrevState = yCurrState;
      telemetry.addData("x curr state/ x prev state", xCurrState + ", " + xPrevState);
      telemetry.addData("y curr state/ y prev state", yCurrState + ", " + yPrevState);
      // convert the RGB values to HSV values.
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

      // send the info back to driver station using telemetry function.
      telemetry.addData("LED", bLedOn ? "On" : "Off");
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.addData("Distance ", colorSensor.getDistance(DistanceUnit.INCH));
      telemetry.addData("Hue", hsvValues[0]);
      telemetry.addData("class", colorSensor.getClass());

      boolean piece_is_there = false;
      final double PIECE_DIST = 3.0;
      if (colorSensor.getDistance(DistanceUnit.INCH)<PIECE_DIST){
        piece_is_there = true;
      }

      telemetry.addData("Is Piece in Intake?:",piece_is_there);

      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      int green = colorSensor.alpha();
      int blue = colorSensor.blue() * 2;
      int red = colorSensor.red();
      if (green > red && green > blue) {
        telemetry.addData("Current Color", "Grey");
        leftPower = 0.33;
        rightPower = 0.33;
      }
      if ((green < red && red > blue)) {
        telemetry.addData("Current Color", "Red");
        leftPower = 0;
        rightPower = 0;
      }
      if (blue > red && green < blue) {
        telemetry.addData("Current Color", "Blue");
        leftPower = 0;
        rightPower = 0;
      }
      if (sensingColor == false) {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
      }
      leftDrive.setPower(leftPower);
      rightDrive.setPower(rightPower);
      relativeLayout.post(new Runnable() {
        @Override
        public void run() {

        }
      });
      {
        //public void run() {
        //}

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
          public void run() {
            relativeLayout.setBackgroundColor(Color.WHITE);
          }
        });
      }
      telemetry.update();
    }
  }}