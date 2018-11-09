package ca.mcgill.ecse211.mountev3rest.navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Provides an interface to move the robot to an arbitrary point on the grid.
 * 
 * The {@code Navigation} enables the robot to go to arbitrary locations on the grid by computing
 * the required distance and angle to reach it from the odometer's readings. Additionally, the class
 * runs on its own thread to enable the caller to change direction at any point if an event occurs.
 * 
 * @author angelortiz
 *
 */
public class Navigation implements Runnable {

  // Class constants
  private static final int FORWARD_SPEED = 150;
  private static final int ROTATE_SPEED = 80;
  private static final int NAVIGATION_PERIOD = 50;
  private static final double TILE_SIZE = 30.48;

  // Class attributes
  // Motors
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // Information about the robot and target
  public final double WHEEL_RADIUS;
  public final double TRACK;
  private Odometer odometer;
  private int[] target;
  private double targetAngle;
  private final double MOTOR_OFFSET;

  // State machine flags
  private boolean directionChanged;
  private boolean isNavigating;
  private boolean GYRO_CORRECTION;

  /**
   * Creates a navigator that will operate using the specified track and wheel radius values.
   * 
   * @param leftMotor Left motor of the robot.
   * @param rightMotor Right motor of the robot.
   * @param wheelRadius Wheel radius of the robot't wheels measured in centimeters.
   * @param track Measurement of the robot's track in centimeter.
   * @throws OdometerException If the singleton {@code Odometer} class has not been instantiated.
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double WHEEL_RADIUS, final double TRACK, final double MOTOR_OFFSET, final boolean GYRO_CORRECTION)
      throws OdometerException {

    this.odometer = Odometer.getOdometer();

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.WHEEL_RADIUS = WHEEL_RADIUS;
    this.TRACK = TRACK;
    this.MOTOR_OFFSET = MOTOR_OFFSET;
    this.GYRO_CORRECTION = GYRO_CORRECTION;

    target = new int[2];
    target[0] = -1;
    target[1] = -1;
    isNavigating = false;
    directionChanged = false;
  }

  /**
   * Runs the state machine main logic. This involves setting the direction of the robot to a new
   * target if required.
   */
  @Override
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Main navigator state machine flow

      // If the direction has changed recompute the trajectory of the robot
      if (directionChanged) {
        goToTarget();
        isNavigating = true;
        directionChanged = false;
      }

      // Correct the trajectory if error angle is too large
      if (GYRO_CORRECTION && isNavigating && Math.abs(targetAngle - odometer.getXYT()[2]) > 5) {
        directionChanged = true;
      }

      // Set this flag to let other threads know that the robot is currently reaching a waypoint
      if (!leftMotor.isMoving() && !rightMotor.isMoving())
        isNavigating = false;

      // This ensures that the navigator only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < NAVIGATION_PERIOD) {
        try {
          Thread.sleep(NAVIGATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  /**
   * Sets current target and indicates the state machine to retrace the trajectory.
   * 
   * @param x New target X position.
   * @param y New target Y position.
   */
  public void travelTo(int x, int y) {
    this.target[0] = x;
    this.target[1] = y;

    directionChanged = true;
    isNavigating = true;
  }

  /**
   * Turns to an absolute angle with respect to the grid ensuring minimal rotation. Positive angles
   * are defined as counter-clockwise rotation and vice-versa.
   * 
   * @param theta Desired angle of rotation.
   */
  public void turnTo(double theta) {
    double currTheta = odometer.getXYT()[2];
    double targetRotation = 0;
    int direction = 1; // 1 for right turn, -1 for left turn

    // Ensure that the minimal turn is taken
    if (theta < currTheta) {
      targetRotation = currTheta - theta;
      if (targetRotation < 180)
        direction = -1;
      else
        targetRotation = 360 - targetRotation;
    } else {
      targetRotation = theta - currTheta;
      if (targetRotation > 180) {
        targetRotation = 360 - targetRotation;
        direction = -1;
      }
    }

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    if (direction == 1) {
      leftMotor.forward();
      rightMotor.backward();
    } else {
      leftMotor.backward();
      rightMotor.forward();
    }

    while (Math.abs(odometer.getXYT()[2] - theta) > 3) {
      try {
        Thread.sleep(50);
      } catch (InterruptedException ex) {
        ex.printStackTrace();
      }
    }

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
  }

  /**
   * Turns to an relative angle with respect to the current position ensuring minimal rotation.
   * Positive angles are defined as counter-clockwise rotation and vice-versa.
   * 
   * @param theta Desired angle of rotation.
   */
  public void turnToRelative(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);
    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), false);
  }

  /**
   * Indicates whether the robot is still navigating.
   * 
   * @return Boolean values indicating if the robot is moving.
   */
  public boolean isNavigating() {
    return isNavigating;
  }

  /**
   * Moves the robot in the direction of the current target set in the state machine.
   */
  private void goToTarget() {
    // Compute the target's absolute angle and the distance required to reach it
    double[] position = odometer.getXYT();
    double[] realTarget =
        computeRealTarget(position[0], position[1], target[0] * TILE_SIZE, target[1] * TILE_SIZE);

    // Turn to target angle
    targetAngle = realTarget[1];
    turnTo(realTarget[1]);

    // Move forward the required target distance
    leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate((int) (convertDistance(WHEEL_RADIUS, realTarget[0]) * MOTOR_OFFSET), true);
    rightMotor.rotate(convertDistance(WHEEL_RADIUS, realTarget[0]), true);
  }

  /**
   * Computes the absolute angle and distance in centimeters required to reach the target with
   * respect to the current position.
   * 
   * @param currX Current X position in centimeters.
   * @param currY Current Y position in centimeters.
   * @param targetX Target X position in centimeters.
   * @param targetY Target Y position in centimeters.
   * @return Array containing the distance and angle required to reach the target in that order.
   */
  private double[] computeRealTarget(double currX, double currY, double targetX, double targetY) {
    double deltaX = targetX - currX;
    double deltaY = targetY - currY;
    int quadrant = 0;
    double[] computedTarget = new double[2];

    // Determine the quadrant of the target with respect to the current position
    if (deltaX >= 0 && deltaY >= 0)
      quadrant = 1;
    else if (deltaX >= 0 && deltaY < 0)
      quadrant = 2;
    else if (deltaX < 0 && deltaY < 0)
      quadrant = 3;
    else if (deltaX < 0 && deltaY >= 0)
      quadrant = 4;

    // Distance to the target
    double distance = Math.hypot(deltaX, deltaY);

    // Compute the absolute angle of direction to the target
    deltaX = Math.abs(deltaX);
    deltaY = Math.abs(deltaY);

    double targetTheta = 0;
    switch (quadrant) {
      case 1:
        targetTheta = Math.toDegrees(Math.atan(deltaX / deltaY));
        break;
      case 2:
        targetTheta = Math.toDegrees(Math.atan(deltaY / deltaX));
        targetTheta += 90;
        break;
      case 3:
        targetTheta = Math.toDegrees(Math.atan(deltaX / deltaY));
        targetTheta += 180;
        break;
      case 4:
        targetTheta = Math.toDegrees(Math.atan(deltaY / deltaX));
        targetTheta += 270;
        break;
    }

    computedTarget[0] = distance;
    computedTarget[1] = targetTheta;

    return computedTarget;
  }

  /**
   * Converts a value distance to the equivalent degrees of rotation in the wheels. Wheel radius and
   * distance should be in the same units.
   * 
   * @param radius Wheel radius to use for conversion.
   * @param distance Distance to convert.
   * @return
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * Computes the required distance travel of each wheel assuming opposite directions to achieve a
   * certain degree of rotation.
   * 
   * @param radius Wheel radius.
   * @param width Width of the robot.
   * @param angle Desired angle of rotation.
   * @return
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
