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

package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@TeleOp(name = "ServoTestConeUpright", group = "Tests")

public class ServoTestConeUpright extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    double pos = .5;
    char leftRight = 'L';
    double rightServoPos = -1;
    double leftServoPos = -1;
    double gamepadRepeatSeconds = .2;

    @Override

    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, true);

        waitForStart();

        while (opModeIsActive()) {

            if (leftRight == 'L') {
                if (pos != leftServoPos) {
                    Robot.coneUprightLeftServo.setPosition(pos);
                    leftServoPos = pos;
                }
            } else {
                if (pos != rightServoPos) {
                    Robot.coneUprightRightServo.setPosition(pos);
                    rightServoPos = pos;
                }
            }

            if (gamepad1.right_bumper && timer.seconds() > gamepadRepeatSeconds) {
                pos = pos + .01;
                timer.reset();
            }

            if (gamepad1.left_bumper && timer.seconds() > gamepadRepeatSeconds) {
                pos = pos - .01;
                timer.reset();
            }

            if (gamepad1.dpad_up && timer.seconds() > gamepadRepeatSeconds) {
                timer.reset();
                if (leftRight == 'L') {
                    leftRight = 'R';
                    pos = rightServoPos;
                    if (pos == -1) {pos = .5;}
                } else {
                    leftRight = 'L';
                    pos = leftServoPos;
                    if (pos == -1) {pos = .5;}
                }
            }

            telemetry.addData("Press Dpad Up to switch servos", leftRight);
            telemetry.addData("Press bumpers to raise/lower pos", pos);
            telemetry.addData("Left Servo pos", leftServoPos);
            telemetry.addData("Right Servo pos", rightServoPos);

            telemetry.update();
        }
    }
}