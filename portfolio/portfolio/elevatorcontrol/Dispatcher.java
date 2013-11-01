/*
18649 Fall 2013
Group 17
Xianle Wang(xianlew)
(other names would go here)
 */
package simulator.elevatorcontrol;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatorcontrol.Utility.CallArray;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

public class Dispatcher extends Controller {

	/***************************************************************************
	 * Declarations
	 **************************************************************************/
	// note that inputs are Readable objects, while outputs are Writeable
	// objects

	// mDoorClosed
	private DoorClosedArray doorClosedFront = new DoorClosedArray(
			Hallway.FRONT, canInterface);
	private DoorClosedArray doorClosedBack = new DoorClosedArray(Hallway.BACK,
			canInterface);
	// mCarLevelPosition
	private ReadableCanMailbox networkCarLevelPosition;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
	// mDriveSpeed
	private ReadableCanMailbox networkDriveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;
	// mDesiredFloor message
	private WriteableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;

	// send mDesiredDwell_b message
	private WriteableCanMailbox networkDesiredDwell_b;
	private IntegerCanPayloadTranslator mDesiredDwell_b;

	// send mDesiredDwell_b message
	private WriteableCanMailbox networkDesiredDwell_f;
	private IntegerCanPayloadTranslator mDesiredDwell_f;

	// store the period for the controller
	private SimTime period;

	// these variables keep track of which instance this is.
	private final int numFloors;
	// additional internal state variables
	private int currentFloor = MessageDictionary.NONE;;
	private int target = 1;
	private Hallway desiredHallway = null;
	private AtFloorArray atFloorArray = null;
	private int dwell = 1000;
	private Direction desiredDir = Direction.STOP;
	private Direction targetDir = Direction.STOP;
	private CallArray callArray;
	// enumerate states
	private enum State {
		OPERATING, EMERGENCY, IDLE
	}

	// state variable initialized to the initial state FLASH_OFF
	private State state = State.IDLE;

	/**
	 * For your elevator controllers, you should make sure that the constructor
	 * matches the method signatures in ControllerBuilder.makeAll().
	 */
	public Dispatcher(int numFloors, SimTime period, boolean verbose) {
		// call to the Controller superclass constructor is required
		super("DispatcherControl", verbose);

		// stored the constructor arguments in internal state
		this.period = period;
		this.numFloors = numFloors;

		log("Created DispatcherControl with period = ", period);
		callArray = new CallArray(canInterface);
		/*
		 * mDesiredFloor
		 */
		networkDesiredFloor = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(
				networkDesiredFloor);
		canInterface.sendTimeTriggered(networkDesiredFloor, period);

		/*
		 * mDesiredDwell_b
		 */
		networkDesiredDwell_b = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID
						+ ReplicationComputer
								.computeReplicationId(Hallway.BACK));
		mDesiredDwell_b = new IntegerCanPayloadTranslator(networkDesiredDwell_b);
		canInterface.sendTimeTriggered(networkDesiredDwell_b, period);
		/*
		 * mCarLevelPosition
		 */
		networkCarLevelPosition = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(
				networkCarLevelPosition);
		canInterface.registerTimeTriggered(networkCarLevelPosition);

		// currentFloor
		atFloorArray = new AtFloorArray(canInterface);
		currentFloor = atFloorArray.getCurrentFloor();

		/*
		 * mDriveSpeed
		 */
		networkDriveSpeed = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.registerTimeTriggered(networkDriveSpeed);

		/*
		 * mDesiredDwell_f
		 */
		networkDesiredDwell_f = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID
						+ ReplicationComputer
								.computeReplicationId(Hallway.FRONT));
		mDesiredDwell_f = new IntegerCanPayloadTranslator(networkDesiredDwell_f);
		canInterface.sendTimeTriggered(networkDesiredDwell_f, period);

		// currentFloor
		atFloorArray = new AtFloorArray(canInterface);
		currentFloor = atFloorArray.getCurrentFloor();

		timer.start(period);
	}

	/*
	 * The timer callback is where the main controller code is executed. For
	 * time triggered design, this consists mainly of a switch block with a case
	 * blcok for each state. Each case block executes actions for that state,
	 * then executes a transition to the next state if the transition conditions
	 * are met.
	 */
	public void timerExpired(Object callbackData) {
		State newState = state;
		switch (state) {
		case IDLE:
			// state var
			currentFloor = atFloorArray.getCurrentFloor();
			// state actions for 'IDLE'
			mDesiredFloor.set(currentFloor, Direction.STOP, Hallway.NONE);
			mDesiredDwell_b.set(dwell);
			mDesiredDwell_f.set(dwell);
			//System.out.println("---callArray.size= "+callArray.size());
// #transition 'T11.4'
			if (atFloorArray.getCurrentFloor() == MessageDictionary.NONE
					&& !(doorClosedBack.getBothClosed() && doorClosedFront
							.getBothClosed())) {
				newState = State.EMERGENCY;
			} else if (callArray.size() > 0) {
// #transition 'T11.1'
				newState = State.OPERATING;
			}
			break;
		case OPERATING:
			// state var
			currentFloor = atFloorArray.getCurrentFloor();
			target = getTarget();
			// state actions for 'OPERATING'
			mDesiredFloor.set(target, desiredDir, desiredHallway);
// #transition 'T11.3'
			if (atFloorArray.getCurrentFloor() == MessageDictionary.NONE
					&& !(doorClosedBack.getBothClosed() && doorClosedFront
							.getBothClosed())) {
				newState = State.EMERGENCY;
			} else if (callArray.size() == 0) {
// #transition 'T11.2'
				newState = State.IDLE;
			}
			break;
		case EMERGENCY:
			target = 1;
            // state actions for 'EMERGENCY'
			mDesiredFloor.set(target, Direction.STOP, Hallway.NONE);
			break;
		default:
			throw new RuntimeException("State " + state
					+ " was not recognized.");
		}

		// log the results of this iteration
		if (state == newState) {
			log("remains in state: ", state, ", disiredFloor:d,f,b:  ",
					mDesiredFloor.getDirection(), mDesiredFloor.getFloor(),
					mDesiredFloor.getHallway());
		} else {
			log("Transition:", state, "->", newState, "disiredFloor:d,f,b:  ",
					mDesiredFloor.getDirection(), mDesiredFloor.getFloor(),
					mDesiredFloor.getHallway());
		}

		// update the state variable
		state = newState;

		// report the current state
		setState(STATE_KEY, newState.toString());

		// schedule the next iteration of the controller
		// you must do this at the end of the timer callback in order to restart
		// the timer
		timer.start(period);
	}

	private int getTarget() {
		Direction tmpDir = getTmpDir();
		List<Integer> floors = getCommitFloorList(tmpDir);
		if(tmpDir.equals(Direction.STOP)){// choose any one)
			for (int i : floors) {
				if (callArray.isCalled(i, Direction.UP)){
					desiredHallway = callArray.getHallway(i, Direction.UP);
					desiredDir = Direction.UP;
					return i;
				}else if(callArray.isCalled(i, Direction.DOWN)) {
					desiredHallway = callArray.getHallway(i, Direction.DOWN);
					desiredDir = Direction.DOWN;
					return i;
				}
			}
			return currentFloor;
		}
		int sameDirTarget = 9;
		for (int i : floors) {
			if (callArray.isCalled(i, tmpDir)) {// choose the nearest
				if ((tmpDir == Direction.UP && sameDirTarget > i)
						|| (tmpDir == Direction.DOWN && sameDirTarget < i))
					sameDirTarget = i;
			}
		}
		if (sameDirTarget != 9) {
			desiredDir = tmpDir;
			desiredHallway = callArray.getHallway(sameDirTarget, tmpDir);
			return sameDirTarget;
		}
		// revDirTarget;
		Direction revDir = reverseDir(tmpDir);
		int revDirTarget = 9;
		for (int i : floors) {
			if (callArray.isCalled(i, tmpDir)) {// choose the farthest
				if ((tmpDir == Direction.UP && revDirTarget < i)
						|| (tmpDir == Direction.DOWN && revDirTarget > i))
					revDirTarget = i;
			}
		}
		if (revDirTarget != 9) {
			desiredDir = revDir;
			desiredHallway = callArray.getHallway(revDirTarget, revDir);
			return revDirTarget;
		}
		// no call at the desired direction,
		throw new RuntimeException(
				"no call at the desired direction\ndesiredDir = " + desiredDir
						+ "\ncurr position = "
						+ mCarLevelPosition.getPosition());
	}

	private Direction getTmpDir() {
		Direction tmpDir = Direction.STOP;
		//System.out.println("speed:----------------"+mDriveSpeed.getSpeed()+mDriveSpeed.getDirection());
		if (mDriveSpeed.getSpeed()>0.1 ) {//!= Speed.LEVEL&& mDriveSpeed.getSpeed() != Speed.STOP
			tmpDir = mDriveSpeed.getDirection();
			targetDir = tmpDir;
			//System.out.println("-----set to "+tmpDir);
		} else {
			//System.out.println("read dir "+targetDir);
			tmpDir = targetDir;
		}
		return tmpDir;
	}

	private Direction reverseDir(Direction tmpDir) {
		if (tmpDir.equals(Direction.UP)) {
			return Direction.DOWN;
		}
		if (tmpDir.equals(Direction.UP)) {
			return Direction.DOWN;
		}
		return Direction.STOP;
	}

	private List<Integer> getCommitFloorList(Direction tmpDir) {
		List<Integer> out = new ArrayList<Integer>();
		int pos = mCarLevelPosition.getPosition();
		int dist = 0;
		double speed = mDriveSpeed.getSpeed();
		if (speed>0.8) {
			dist = 1000;
		} else {
			dist = 0;
		}
		if (tmpDir.equals(Direction.DOWN)) {
			int tmp = pos - dist;
			//System.out.println("commit point: " + tmp);

			for (int i = 1; i * 5000 - 50 <= tmp; ++i) {
				out.add(i);
			}
		} else if (tmpDir.equals(Direction.UP)) {
			int tmp = pos + dist;
			//System.out.println("commit point: " + tmp);

			for (int i = 8; (i * 5000 + 50 >= tmp)&&i>0; --i) {
				out.add(i);
			}
		} else {
			for (int i = 1; i <= 8; ++i) {
				out.add(i);
			}
		}
		Collections.sort(out);
		return out;
	}

//	private void setDesiredHallway() {
//		int f = currentFloor;
//		if (f == MessageDictionary.NONE) {
//			desiredHallway = Hallway.NONE;
//			return;
//		}
//		if (atFloorArray.isAtFloor(f, Hallway.FRONT)) {
//			if (atFloorArray.isAtFloor(f, Hallway.BACK)) {
//				desiredHallway = Hallway.BOTH;
//			} else {
//				desiredHallway = Hallway.FRONT;
//			}
//		} else if (atFloorArray.isAtFloor(f, Hallway.BACK)) {
//			desiredHallway = Hallway.BACK;
//		} else {
//			desiredHallway = Hallway.NONE;
//		}
//	}
//
//	private Direction getDesiredDirectoin(int target) {
//		if (atFloorArray.getCurrentFloor() < Elevator.numFloors) {
//			return Direction.UP;
//		} else {
//			return Direction.DOWN;
//		}
//	}
}
