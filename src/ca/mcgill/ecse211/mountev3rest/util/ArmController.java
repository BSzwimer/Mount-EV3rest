package ca.mcgill.ecse211.mountev3rest.util;

import ca.mcgill.ecse211.mountev3rest.sensor.ColorDetector;
import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Provides a set of methods to perform the basic subtasks required for ring collection.
 * 
 * Among the facilities provided by the {@code ArmController} class are ring identification, and
 * ring retrieval.
 * 
 * @author angelortiz
 *
 */
public class ArmController {

  private EV3LargeRegulatedMotor motor1;
  private EV3LargeRegulatedMotor motor2;
  private UltrasonicPoller usPoller;
  private LightPoller lightPoller;
  private ColorDetector colorDetector;

  /**
   * Creates an {@code ArmController} that will operate on the provided motors.
   * 
   * @param motor1 Motor that will be used to control the height of the base on the arm.
   * @param motor2 Motor that will be used to control the displacement of the base on the arm.
   */
  public ArmController(EV3LargeRegulatedMotor motor1, EV3LargeRegulatedMotor motor2) {
    // TODO
  }

  /**
   * Moves the arm to one of three predefined positions. Each of the positions provides the robot
   * with a different total height or length which might be required to perform a particular task
   * like crossing a tunnel. The encoding of the positions is as follows:
   * <ul>
   *    <li>2 = High</li>
   *    <li>1 = Medium</li>
   *    <li>0 = Low</li>
   * </ul>
   * 
   * @param height Position to set the arm to.
   */
  public void ajustHeight(int height) {
    // TODO
  }

  /**
   * Swipes the arm across the height of one the tree's faces and identifies any rings and their
   * color. This method assumes that the robot is already facing directly into the tree face to
   * analyze when it is called.
   * 
   * @return A two element list with color of the ring found in the high and low level of the tree
   *         face. Color encoding is as defined in the {@code ColorDetector} class.
   * @see ColorDetector
   */
  public int[] identifyRings() {
    // TODO
    return new int[1];
  }

  /**
   * Gets a ring from the tree face that the robot is looking at when the method is called. This
   * method assumes that the robot is already properly aligned with the corresponding tree face. A
   * number of beeps is produced before the ring is grabbed corresponding to the color of the ring
   * as defined by the {@code ColorDetector}'s encoding.
   * 
   * @param position Position of the ring to get, 1 for high or 0 for low.
   * @see ColorDetector
   */
  public void getRing(int position) {
    // TODO
  }

  /**
   * Releases the ring being carried by the robot on the arm if any.
   */
  public void releaseRing() {
    // TODO
  }

}
