package ca.mcgill.ecse211.mountev3rest.controller;

import ca.mcgill.ecse211.mountev3rest.navigation.Display;
import ca.mcgill.ecse211.mountev3rest.navigation.Localizer;
import ca.mcgill.ecse211.mountev3rest.navigation.Navigation;
import ca.mcgill.ecse211.mountev3rest.navigation.Odometer;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometerException;
import ca.mcgill.ecse211.mountev3rest.sensor.ColorDetector;
import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.mountev3rest.util.ArmController;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Provides an interface to execute the main subtasks required for the robot to perform the overall
 * routine.
 * <p>
 * The {@code DomainController} represents the low layer of the controller in the system. The class
 * is not concerned with the general state of the main routine execution. Instead, the
 * {@code DomainController} handles all the details related to each of the subtasks required for the
 * main routine. This means that internally, the {@code DomainCotroller} handles all the required
 * calls to more specialized classes and ensures that the requested subtask is completed.
 * <p>
 * It is assumed that the caller of the {@code DomainController} methods (usually the
 * {@code MetaController}) will conform to the external requirements of each subtask. For instance,
 * a call to {@code crossTunnel(3, 3, 4, 6)} will not produce the expected outcome if the odometer
 * of the robot has not been set to the right initial values through the {@code localize()} method.
 * 
 * @see MetaController
 * 
 * @author angelortiz
 *
 */
public class DomainController {

  // Constants
  private static final int STANDARD_WAIT = 50;
  private static final double TILE_SIZE = 30.48;
  private static final double MOTOR_OFFSET = 1.01; // TEST BATTERY IS 8V FOR BEST VALUE.
  private static final double SENSOR_OFFSET = 12.5; // MAYBE TEST?
  private static final double SENSOR_OFFSET_ANGLE = 17.82; // ALSO THIS?
  private static final boolean GYRO_PATH_CORRECTION = false;

  // Parameters
  private final double WHEEL_RADIUS;
  private final double TRACK;

  // Attributes
  Odometer odometer;
  Navigation navigation;
  Localizer localizer;
  UltrasonicPoller usPoller;
  LightPoller lightPoller;
  ArmController armController;

  // Temporal
  EV3LargeRegulatedMotor leftMotor;
  EV3LargeRegulatedMotor rightMotor;

  // Threads
  Thread odoThread;
  Thread navThread;
  Thread lightThread;

  /**
   * Creates a {@code DomainController} and initializes all the required specialized classes.
   * <p>
   * Instantiation of the {@code DomainController} involves initializing the singleton
   * {@code Odometer}, {@code LightPoller}, and {@code UltrasonicPoller} classes, which is required
   * for the instantiation of other classes.
   * 
   * @throws OdometerException If there is a problem while creating the Odometer singleton instance.
   * @throws PollerException
   * 
   * @see Odometer
   * @see LightPoller
   * @see UltrasonicPoller
   */
  public DomainController(final double TRACK, final double WHEEL_RADIUS)
      throws OdometerException, PollerException {
    // Set parameter values
    this.TRACK = TRACK;
    this.WHEEL_RADIUS = WHEEL_RADIUS;

    // Get motor objects
    leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    // EV3LargeRegulatedMotor armLeftMotor = new
    // EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
    // EV3LargeRegulatedMotor armRightMotor = new
    // EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

    // Instantiate the sensors
    EV3GyroSensor gyroSensor = new EV3GyroSensor(LocalEV3.get().getPort("S1"));
    EV3ColorSensor topLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
    EV3ColorSensor lineLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
    EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));

    // Create the specialized objects
    odometer = Odometer.getOdometer(leftMotor, rightMotor, gyroSensor, WHEEL_RADIUS, MOTOR_OFFSET);
    navigation = new Navigation(leftMotor, rightMotor, WHEEL_RADIUS, TRACK, MOTOR_OFFSET,
        GYRO_PATH_CORRECTION);
    usPoller = UltrasonicPoller.getUltrasonicPoller(usSensor);
    lightPoller = LightPoller.getLightPoller(topLightSensor, lineLightSensor);
    localizer = new Localizer(leftMotor, rightMotor, navigation, SENSOR_OFFSET, SENSOR_OFFSET_ANGLE,
        TILE_SIZE);
    // armController = new ArmController(armLeftMotor, armRightMotor);

    // Initialize and start the required extra threads
    odoThread = new Thread(odometer);
    navThread = new Thread(navigation);
    lightThread = new Thread(lightPoller);
    odoThread.start();
    navThread.start();
    lightThread.start();
  }

  /**
   * Uses the {@code Localizer} class to provide the odometer with an initial set of coordinates
   * that correspond to the location of the robot with respect to the grid.
   * 
   * @see Localizer
   */
  public void localize() {
    // TODO
  }

  /**
   * Crosses the tunnel specified by the given coordinates. The method makes sure that the robot
   * reaches the closest entrance of the tunnel, then it moves through it until the robot is
   * completely outside on the other side.
   * 
   * @param BR_LL_x Lower left X coordinate of the tunnel.
   * @param BR_LL_y Lower left Y coordinate of the tunnel.
   * @param BR_UR_x Upper right X coordinate of the tunnel.
   * @param BR_UR_y Upper right Y coordinate of the tunnel.
   */
  public void crossTunnel(int BR_LL_x, int BR_LL_y, int BR_UR_x, int BR_UR_y) {
    // TODO
  }

  /**
   * Approaches the tree given by the coordinates and positions the robot looking straight into a
   * particular face of the tree. The faces of the tree are defined by the range [0, 3] counting
   * counter-clockwise and starting from the face looking South.
   * 
   * @param T_LL_x Lower left X coordinate of the tree.
   * @param T_LL_y Lower left Y coordinate of the tree.
   * @param T_UR_x Upper right X coordinate of the tree.
   * @param T_UR_y Upper right Y coordinate of the tree.
   * @param face Face of the tree towards which the robot will be positioned.
   */
  public void approachTree(int T_LL_x, int T_LL_y, int T_UR_x, int T_UR_y, int face) {
    // TODO
  }

  /**
   * Sends the robot to each face of the tree and uses the robot's arm to identify the positions of
   * each ring. A ring map is populated in a 4 by 2 array, where the first dimension represents the
   * face where the ring is, and the second dimension represents the height of the ring on the tree
   * face. The contents of the array are integers matching the encoding of the {@code ColorDetector}
   * class.
   * <p>
   * The encoding of the faces is the range {@code [0-3]} where 0 is the face looking South and the
   * rest are defined in the counter-clockwise direction. The height encoding is {@code [0-1]} where
   * 1 is high and 0 is low. For instance,
   * <p>
   * {@code ringMap[2][0] = 2}
   * <p>
   * Represents that the ring on the face looking North, on the low level is green.
   * 
   * @param ringMap 4 by 2 array to store the ring map.
   * @see ColorDetector
   * @see ArmController
   */
  public void identifyRings(int[][] ringMap) {
    // TODO
  }

  /**
   * Computes the optimal ring target using the provided ring map and grabs it using the robot's
   * arm. The ring map should be a 4 by 2 array, where the first dimension represents the face where
   * the ring is, and the second dimension represents the height of the ring on the tree face. The
   * contents of the array are integers matching the encoding of the {@code ColorDetector} class.
   * <p>
   * The encoding of the faces is the range {@code [0-3]} where 0 is the face looking South and the
   * rest are defined in the counter-clockwise direction. The height encoding is {@code [0-1]} where
   * 1 is high and 0 is low. For instance,
   * <p>
   * {@code ringMap[1][1] = 4}
   * <p>
   * Represents that the ring on the face looking East, on the high level is orange.
   * 
   * @param ringMap Ring locations on the tree, used to compute optimal ring target.
   * @see ColorDetector
   * @see ArmController
   */
  public void grabRings(int[][] ringMap) {
    // TODO
  }

  /**
   * TODO
   * 
   * @throws OdometerException If the odometer has not been instantiated.
   */
  public void testNavigation(int[][] points, TextLCD lcd)
      throws OdometerException {
    Display display = new Display(lcd);
    Thread disThread = new Thread(display);
    disThread.start();

    localizer.ultrasonicLocalization();

    localizer.lightLocalization(1, 3, 1);

    for (int[] point : points) {
      navigation.travelTo(point[0], point[1]);
      while (navigation.isNavigating()) {
        try {
          Thread.sleep(STANDARD_WAIT);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }

  }

}
