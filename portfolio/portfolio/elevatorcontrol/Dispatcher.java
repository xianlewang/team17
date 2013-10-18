/*
18649 Fall 2013
Group 17
Xianle Wang(xianlew)
(other names would go here)
 */
package simulator.elevatorcontrol;
import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

public class Dispatcher extends Controller {

	/***************************************************************************
	 * Declarations
	 **************************************************************************/
	// note that inputs are Readable objects, while outputs are Writeable
	// objects

	//mDoorClosed
	private DoorClosedArray doorClosedFront = new DoorClosedArray(Hallway.FRONT, canInterface);
	private DoorClosedArray doorClosedBack = new DoorClosedArray(Hallway.BACK, canInterface);
	
	// send mDesiredFloor message
	private WriteableCanMailbox networkDesiredFloor;
	// translator for the at floor message
	private DesiredFloorCanPayloadTranslator mDesiredFloor;

	// send mDesiredDwell_b message
	private WriteableCanMailbox networkDesiredDwell_b;
	// translator for the at floor message
	private IntegerCanPayloadTranslator mDesiredDwell_b;

	// send mDesiredDwell_b message
	private WriteableCanMailbox networkDesiredDwell_f;
	// translator for the at floor message
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
    private Direction lastDir = Direction.STOP;
	// enumerate states
	private enum State {
		MOVE, EMERGENCY_MOVE, HOLD
	}

	// state variable initialized to the initial state FLASH_OFF
	private State state = State.HOLD;

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

		/*
		 * The log() method is inherited from the Controller class. It takes an
		 * array of objects which will be converted to strings and concatenated
		 * only if the log message is actually written.
		 * 
		 * For performance reasons, call with comma-separated lists, e.g.:
		 * log("object=",object); Do NOT call with concatenated objects like:
		 * log("object=" + object);
		 */
		log("Created DispatcherControl with period = ", period);

		/*
		 * mDesiredFloor
		 */
		// initialize network interface
		networkDesiredFloor = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		// initialize a translator
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(
				networkDesiredFloor);
		// register with network interface to receive periodic update
		canInterface.sendTimeTriggered(networkDesiredFloor, period);

		/*
		 * mDesiredDwell_b
		 */
		// initialize network interface
		networkDesiredDwell_b = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID
						+ ReplicationComputer
								.computeReplicationId(Hallway.BACK));
		// initialize a translator
		mDesiredDwell_b = new IntegerCanPayloadTranslator(networkDesiredDwell_b);
		// register with network interface to send periodic update
		canInterface.sendTimeTriggered(networkDesiredDwell_b, period);

		// currentFloor
		atFloorArray = new AtFloorArray(canInterface);
		currentFloor = atFloorArray.getCurrentFloor();

		/*
		 * mDesiredDwell_f
		 */
		// initialize network interface
		networkDesiredDwell_f = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID
						+ ReplicationComputer
								.computeReplicationId(Hallway.FRONT));
		// initialize a translator
		mDesiredDwell_f = new IntegerCanPayloadTranslator(networkDesiredDwell_f);
		// register with network interface to send periodic update
		canInterface.sendTimeTriggered(networkDesiredDwell_f, period);

		// currentFloor
		atFloorArray = new AtFloorArray(canInterface);
		currentFloor = atFloorArray.getCurrentFloor();

		/*
		 * issuing the timer start method with no callback data means a NULL
		 * value will be passed to the callback later. Use the callback data to
		 * distinguish callbacks from multiple calls to timer.start() (e.g. if
		 * you have multiple timers.
		 */
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
		case HOLD:
			// state var
			currentFloor = atFloorArray.getCurrentFloor();
			setDesiredHallway();

			// state actions for 'HOLD'
			mDesiredFloor.set(target, lastDir, desiredHallway);
			mDesiredDwell_b.set(dwell);
			mDesiredDwell_f.set(dwell);
// #transition 'T11.1'
			if (atFloorArray.getCurrentFloor() == MessageDictionary.NONE
					&& !(doorClosedBack.getBothClosed() && doorClosedFront.getBothClosed())) {
				newState = State.EMERGENCY_MOVE;
// #transition 'T11.3'
			} else if (atFloorArray.getCurrentFloor() == mDesiredFloor
					.getFloor()
					&& !(doorClosedBack.getBothClosed() && doorClosedFront.getBothClosed())) {
				newState = State.MOVE;
			}
			break;
		case MOVE:
			// state var
			currentFloor = atFloorArray.getCurrentFloor();
			setDesiredHallway();
			// state actions for 'HOLD'
			target = currentFloor % numFloors + 1;
            lastDir = getDesiredDirectoin();
			mDesiredFloor.set(target, getDesiredDirectoin(), desiredHallway);

// #transition 'T11.2'
			if (currentFloor != target) {
				newState = State.HOLD;
			}
			break;
		case EMERGENCY_MOVE:
			target = 1;
			mDesiredFloor.set(target, Direction.STOP, Hallway.NONE);
			break;
		default:
			throw new RuntimeException("State " + state
					+ " was not recognized.");
		}

		// log the results of this iteration
		if (state == newState) {
			log("remains in state: ", state, ", disiredFloor:d,f,b:  ",mDesiredFloor.getDirection(),mDesiredFloor.getFloor(),mDesiredFloor.getHallway());
		} else {
			log("Transition:", state, "->", newState, "disiredFloor:d,f,b:  ",mDesiredFloor.getDirection(),mDesiredFloor.getFloor(),mDesiredFloor.getHallway());
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

	private void setDesiredHallway() {
		int f = currentFloor;
		if (f == MessageDictionary.NONE) {
			desiredHallway = Hallway.NONE;
			return;
		}
		if (atFloorArray.isAtFloor(f, Hallway.FRONT)) {
			if (atFloorArray.isAtFloor(f, Hallway.BACK)) {
				desiredHallway = Hallway.BOTH;
			} else {
				desiredHallway = Hallway.FRONT;
			}
		} else if (atFloorArray.isAtFloor(f, Hallway.BACK)) {
			desiredHallway = Hallway.BACK;
		} else {
			desiredHallway = Hallway.NONE;
		}
	}
	private Direction getDesiredDirectoin(){
		if(atFloorArray.getCurrentFloor()<Elevator.numFloors){
			return Direction.UP;
		}else{
			return Direction.DOWN;
		}
	}
}
