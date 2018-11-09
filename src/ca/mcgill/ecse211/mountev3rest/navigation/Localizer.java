package ca.mcgill.ecse211.mountev3rest.navigation;

import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.sensor.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Provides localization functionality, which allows the robot to compute its initial location when
 * placed on the grid.
 * <p>
 * The {@code Localizator} class combines ultrasonic and light sensor localization into a single
 * routine that corrects the X, Y and Theta values of the odometer to the location of the robot with
 * respect to the grid.
 * 
 * @author angelortiz
 *
 */
public class Localizer {

  // Constants
  private static final int LOCALIZATION_PERIOD = 25;
  private static final int ROTATE_SPEED = 50;
  private static final int THRESHOLD = 40;
  private static final double HALF_TILE_APPROX = 0.4;
  private final double SENSOR_OFFSET;
  private final double SENSOR_OFFSET_ANGLE;
  private final double TILE_SIZE;

  // Object Attributes
  private Odometer odometer;
  private UltrasonicPoller usPoller;
  private LightPoller lightPoller;
  private Navigation navigation;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // Localization Attributes
  private int prevDistance;
  private int currDistance;
  private boolean measurementTaken;
  private double alpha;
  private double beta;

  /**
   * Creates a {@code Localizator} that will operate on the singleton {@code Odometer} class.
   * 
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   * @throws PollerException
   * @see Odometer
   */
  public Localizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      Navigation navigation, final double SENSOR_OFFSET, final double SENSOR_OFFSET_ANGLE,
      final double TILE_SIZE) throws OdometerException, PollerException {
    usPoller = UltrasonicPoller.getUltrasonicPoller();
    lightPoller = LightPoller.getLightPoller();
    odometer = Odometer.getOdometer();

    this.navigation = navigation;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.SENSOR_OFFSET = SENSOR_OFFSET;
    this.SENSOR_OFFSET_ANGLE = SENSOR_OFFSET_ANGLE;
    this.TILE_SIZE = TILE_SIZE;
  }

  /**
   * Performs falling edge ultrasonic localization to correct the angle Theta of the odometer and
   * then it uses light localization to get the correct values of the X and Y coordinates of the
   * robot's location. The values obtained are used to overwrite the odometer's current values.
   */
  public void localize(int referenceCorner, int refX, int refY) {
    ultrasonicLocalization();
    lightLocalization(referenceCorner, refX, refY);
  }

  /**
   * Estimates the angle of the robot with respect to the grid by rotating until a falling edge is
   * detected with the wall on each side. The angle in the middle of the two falling edge detections
   * is used to estimate the robot's real angle.
   */
  public void ultrasonicLocalization() {
    long updateStart, updateEnd;
    prevDistance = -1;
    boolean firstSearch = true;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();

    // Falling edge localization
    while (true) {
      updateStart = System.currentTimeMillis();
      currDistance = usPoller.poll();
      if (prevDistance < 0) {
        prevDistance = currDistance;
        continue;
      }
      if (prevDistance >= currDistance && prevDistance > THRESHOLD) { // Distance increasing
        if (!measurementTaken && currDistance < THRESHOLD) {
          if (firstSearch) {
            beta = odometer.getXYT()[2];
            leftMotor.backward();
            rightMotor.forward();
            firstSearch = false;
            measurementTaken = true;
          } else {
            alpha = odometer.getXYT()[2];
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            break;
          }
        }
      } else if (prevDistance < currDistance) { // Distance decreasing
        measurementTaken = false;
      }
      prevDistance = currDistance;
      // This ensures that the navigator only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < LOCALIZATION_PERIOD) {
        try {
          Thread.sleep(LOCALIZATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
    correctAngle();
  }

  /**
   * Updates the X and Y values of the odometer by moving forward in the +X and +Y directions until
   * a line is detected using the light sensors. This method assumes that the angle of the odometer
   * can be trusted to some small degree of error.
   */
  public void lightLocalization(int referenceCorner, int refX, int refY) {
    boolean inLine = false;
    int lineDetections = 0;
    double[] angleDetection = new double[4]; // Array to store the angle detections
    int approxAngle = -1;

    // Assume the robot is around the middle of the tile, move towards the closest corner.
    switch(referenceCorner) {
      case 0:
        approxAngle = 45;
        break;
      case 1:
        approxAngle = 315;
        break;
      case 2:
        approxAngle = 225;
        break;
      case 3:
        approxAngle = 135;
        break;
    }
    
    navigation.turnTo(approxAngle);

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(
        Navigation.convertDistance(navigation.WHEEL_RADIUS, TILE_SIZE * HALF_TILE_APPROX), true);
    rightMotor.rotate(
        Navigation.convertDistance(navigation.WHEEL_RADIUS, TILE_SIZE * HALF_TILE_APPROX), false);

    // Rotate counter-clockwise and record 4 angles at line detections.
    leftMotor.backward();
    rightMotor.forward();

    lightPoller.enableBeeping();
    while (lineDetections < 4) {
      if (lightPoller.inLine) {
        if (!inLine) {
          angleDetection[lineDetections] = odometer.pollGyro();
          lineDetections++;
          inLine = true;
        }
      } else {
        inLine = false;
      }
    }
    lightPoller.disableBeeping();

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    // Correct the odometer using the angles of the line detections.
    double nX = angleDetection[3]; // Negative X intersection
    double pX = angleDetection[1]; // Positive X intersection
    double nY = angleDetection[0]; // Negative Y intersection
    double pY = angleDetection[2]; // Positive Y intersection

    double thetaY = pY - nY;
    double thetaX = nX - pX;

    double correctedX = 0;
    double correctedY = 0;
    double correctedTheta = 0;
    switch (referenceCorner) {
      case 0:
        correctedX = TILE_SIZE * refX - SENSOR_OFFSET * Math.cos(Math.toRadians(thetaY) / 2);
        correctedY = TILE_SIZE * refY - SENSOR_OFFSET * Math.cos(Math.toRadians(thetaX) / 2);
        correctedTheta = 180 - (thetaX / 2) + SENSOR_OFFSET_ANGLE;
        break;
      case 1:
        correctedY = TILE_SIZE * refY - SENSOR_OFFSET * Math.cos(Math.toRadians(thetaY) / 2);
        correctedX = TILE_SIZE * refX + SENSOR_OFFSET * Math.cos(Math.toRadians(thetaX) / 2);
        correctedTheta = 90 - (thetaX / 2) + SENSOR_OFFSET_ANGLE;
        break;
      case 2:
        correctedX = TILE_SIZE * refX + SENSOR_OFFSET * Math.cos(Math.toRadians(thetaY) / 2);
        correctedY = TILE_SIZE * refY + SENSOR_OFFSET * Math.cos(Math.toRadians(thetaX) / 2);
        correctedTheta = 0 - (thetaX / 2) + SENSOR_OFFSET_ANGLE;
        break;
      case 3:
        correctedY = TILE_SIZE * refY + SENSOR_OFFSET * Math.cos(Math.toRadians(thetaY) / 2);
        correctedX = TILE_SIZE * refX - SENSOR_OFFSET * Math.cos(Math.toRadians(thetaX) / 2);
        correctedTheta = 270 - (thetaX / 2) + SENSOR_OFFSET_ANGLE;
        break;
    }

    odometer.setXYT(correctedX, correctedY, correctedTheta);
  }
  
  /**
   * Uses angle measurements from rising or falling edge methods to correct the odometer's theta
   * reading.
   */
  private void correctAngle() {
    double[] position = odometer.getXYT();
    double correctedTheta;
    if (alpha < beta) {
      correctedTheta = position[2] + 45 - ((alpha + beta) / 2);
    } else {
      correctedTheta = position[2] + 225 - ((alpha + beta) / 2);
    }
    odometer.setTheta(correctedTheta);
  }

}
