package org.firstinspires.ftc.teamcode;

public class Odometry_v1 {
   import com.qualcomm.robotcore.hardware.DcMotor;
   import com.qualcomm.robotcore.hardware.HardwareMap;


        private DcMotor leftEncoder, rightEncoder;
        private double wheelDiameter;
        private double ticksPerRevolution;
        private double encoderTicksPerInch;

        private double xPosition = 0; // Current x-position
        private double yPosition = 0; // Current y-position
        private double heading = 0;   // Current heading (in degrees)

        private double previousLeftTicks = 0;
        private double previousRightTicks = 0;

        public Odometry(HardwareMap hardwareMap, double wheelDiameter, double ticksPerRevolution, double trackWidth) {
            this.wheelDiameter = wheelDiameter;
            this.ticksPerRevolution = ticksPerRevolution;

            leftEncoder = hardwareMap.dcMotor.get("left_encoder");
            rightEncoder = hardwareMap.dcMotor.get("right_encoder");

            // Set the motor run mode to RUN_USING_ENCODER
            leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Calculate the number of encoder ticks per inch
            encoderTicksPerInch = ticksPerRevolution / (wheelDiameter * Math.PI);

            // Set the track width (distance between left and right wheels)
            // You may need to measure and adjust this value for your specific robot
            double trackWidthInches = trackWidth; // Update with your track width in inches

            // Calculate the number of encoder ticks per degree of rotation
            double encoderTicksPerDegree = (Math.PI * trackWidthInches * encoderTicksPerInch) / 360.0;
        }

        public void updatePosition() {
            double leftTicks = leftEncoder.getCurrentPosition();
            double rightTicks = rightEncoder.getCurrentPosition();

            // Calculate the change in ticks for each wheel
            double deltaLeftTicks = leftTicks - previousLeftTicks;
            double deltaRightTicks = rightTicks - previousRightTicks;

            // Calculate the distance traveled by each wheel in inches
            double leftDistance = deltaLeftTicks / encoderTicksPerInch;
            double rightDistance = deltaRightTicks / encoderTicksPerInch;

            // Calculate the change in heading (angle) in degrees
            double deltaHeading = (leftDistance - rightDistance) / trackWidth;

            // Update the robot's position and heading
            xPosition += (leftDistance + rightDistance) / 2.0 * Math.cos(Math.toRadians(heading + deltaHeading / 2.0));
            yPosition += (leftDistance + rightDistance) / 2.0 * Math.sin(Math.toRadians(heading + deltaHeading / 2.0));
            heading += deltaHeading;

            // Store the current ticks for the next update
            previousLeftTicks = leftTicks;
            previousRightTicks = rightTicks;
        }

        public double getXPosition() {
            return xPosition;
        }

        public double getYPosition() {
            return yPosition;
        }

        public double getHeading() {
            return heading;
        }

}
