/*
 18649 Fall 2013
 Group 17
 Qiang Zhang(qiangz)
 Shen Yu (sheny), Jiang He (jiangh), Xianle Wang (xianlew)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.CallArray;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.CarLevelPositionPayload.ReadableCarLevelPositionPayload;
import simulator.payloads.CarPositionIndicatorPayload.ReadableCarPositionIndicatorPayload;
import simulator.payloads.CarPositionPayload.ReadableCarPositionPayload;
import simulator.payloads.CarWeightPayload.ReadableCarWeightPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.DrivePayload.ReadableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;


public class Proj11RuntimeMonitor extends RuntimeMonitor {

    DoorStateMachine doorState = new DoorStateMachine();
    WeightStateMachine weightState = new WeightStateMachine();
    HallCallStateMachine hallcallState = new HallCallStateMachine();
    CarCallStateMachine  carcallState = new CarCallStateMachine();
    ReversalStateMachine reversalState = new ReversalStateMachine();
    DriveStateMachine driveState = new DriveStateMachine();
    Stopwatch stopWatch[] = new Stopwatch[2];
    
    // Qiang Lantern
    LanternStateMachine lanternState = new LanternStateMachine();
    private boolean lanternUpOn;
    private boolean lanternDownOn;
    private int currentFloor;
    //
    // Qiang Drive
    private double carPosition;
    //
    
    // the overweight part
    boolean hasMoved = false;
    boolean wasOverweight = false;
    int overWeightCount = 0;
    // the no call part
    boolean[] noHallCall = new boolean[2];
    boolean[] noCarCall  = new boolean[2];
    int noCallCount = 0;
    boolean[][][] HallCall = new boolean[8][2][2];
    boolean[][] CarCall = new boolean[8][2];
    // the reversal part
    boolean isReversal = false;
    boolean startCountReversal = false;
    double totalReversalCount = 0;
    int[][] tmpReversalCount = new int[2][2];
    public Proj11RuntimeMonitor() {
    	stopWatch[0] = new Stopwatch();
    	stopWatch[1] = new Stopwatch();
    	for (int i = 0; i < 8; i++) {
    		HallCall[i][0][0] = false;
    		HallCall[i][0][1] = false;
    		HallCall[i][1][0] = false;
    		HallCall[i][1][1] = false;
    		CarCall[i][0] = false;
    		CarCall[i][1] = false;
    	}
    	noHallCall[0] = false;
    	noHallCall[1] = false;
    	noCarCall[0]  = false;
    	noCarCall[1]  = false;
    	
    	tmpReversalCount[0][0] = 0;
    	tmpReversalCount[0][1] = 0;
    	tmpReversalCount[1][0] = 0;
    	tmpReversalCount[1][1] = 0;
    }

    @Override
    protected String[] summarize() {
        String[] arr = new String[3];
        arr[0] = "Overweight Count = " + overWeightCount;
        arr[1] = "NoCall Count = " + noCallCount;
        arr[2] = "Reversal Count = " + totalReversalCount;  
        return arr;
    }
    @Override
	public void timerExpired(Object callbackData) {
        //do nothing
    }
    /**************************************************************************
     * high level event methods
     *
     * these are called by the logic in the message receiving methods and the
     * state machines
     **************************************************************************/
    /**
     * Called once when the door starts opening
     * @param hallway which door the event pertains to
     */
    private void doorOpening(Hallway hallway) {
    	// if door opens at a floor but no hall call or car call then add 1 to the noCallCount;
    	if (noHallCall[0] && noCarCall[0]) {
    		// R-T7
    		warning("No Call At Front");
    		noCallCount += 1;
    		noHallCall[0] = false;
    		noCarCall[0]  = false;
    	}
    	if (noHallCall[1] && noCarCall[1]) {
    		// R-T7
    		warning("No Hall Call At Back");
    		noCallCount += 1;
    		noHallCall[1] = false;
    		noCarCall[1]  = false;
    	}
    	if (noHallCall[0]) { noHallCall[0] = false; }
    	if (noHallCall[1]) { noHallCall[1] = false; }
    	if (noCarCall[0])  { noCarCall[0]  = false; }
    	if (noCarCall[1])  { noCarCall[1]  = false; }
    }

    /**
     * Called once when the door starts closing
     * @param hallway which door the event pertains to
     */
    private void doorClosing(Hallway hallway) {
    }

    /**
     * Called once if the door starts opening after it started closing but before
     * it was fully closed.
     * @param hallway which door the event pertains to
     */
    private void doorReopening(Hallway hallway) {
    	if (isReversal) {
    		totalReversalCount += 1;
    		message("Reverse");
    		isReversal = false;
    		startCountReversal = true;
        	stopWatch[hallway.ordinal()].start();
    	}
    }

    /**
     * Called once when the doors close completely
     * @param hallway which door the event pertains to
     */
    private void doorClosed(Hallway hallway) {
        //once all doors are closed, check to see if the car was overweight
        if (!doorState.anyDoorOpen()) {
            if (wasOverweight) {
                message("Overweight");
                overWeightCount++;
                wasOverweight = false;
            }
        }
        // stop the stopwatch if the door closed after reversal
        if (startCountReversal) {
        	stopWatch[hallway.ordinal()].stop();
        	totalReversalCount = stopWatch[0].getAccumulatedTime().getFracSeconds() + stopWatch[1].getAccumulatedTime().getFracSeconds();
        }
    }
    
    /**
     * Qiang code
     * Called once when the doors start nudging 
     */
    private void doorNudging(Hallway hallway) {
    	//R-T10
    	if (tmpReversalCount[hallway.ordinal()][0] == 0 && tmpReversalCount[hallway.ordinal()][1] == 0) {
    		warning("More than one revesal signal before ndging" + tmpReversalCount[hallway.ordinal()][0] + " " + tmpReversalCount[hallway.ordinal()][1]);
    	}
    	tmpReversalCount[hallway.ordinal()][0] = 0;
    	tmpReversalCount[hallway.ordinal()][1] = 0;
    }
    
    /**
     * Called once when the doors are fully open
     * @param hallway which door the event pertains to
     */
    private void doorOpened(Hallway hallway) {
    }
    /**
     * Called when the car weight changes
     * @param hallway which door the event pertains to
     */
    private void weightChanged(int newWeight) {
        if (newWeight > Elevator.MaxCarCapacity) {
            wasOverweight = true;
        }
    }
    /*************************************************************************
     * Qiang drive functions
     *************************************************************************/
    private void driveStop() {
    	// R-T6
    	if (!haveCall(currentFloor)) {
    		warning("Car Stop at floor where no call appears");
    	}
    }
    private void driveFast() {
    }
    private void driveSlow() {
    	// R-T9
    	double slowRangeBottom = 5 * (carPositionIndicator.floor() - 1) - 1;
    	double slowRangeTop    = 5 * (carPositionIndicator.floor() - 1) + 1;
    	carPosition = (double) carLevelPosition.position() / 1000.0;
    	if ((carPosition < slowRangeBottom || carPosition > slowRangeTop) && carPosition != 0) {
    		//warning("CarPosition: " + carLevelPosition.position());
    		//warning("Car Floor " + carPositionIndicator.floor());
    		warning("Drive command should be fast rather than slow" + " CurrentFloor is " + carPositionIndicator.floor() + " currentPosition is " + carPositionIndicator.floor());
    	}
    }
    
    /*************************************************************************
     * Qiang lantern functions
     *************************************************************************/
    /**
     * Called when the lantern is off
     */
    private void lanternOff() {
    	// reset lanternUpOn and lanternDownOn
    	lanternUpOn = false;
    	lanternDownOn = false;
    }
    /**
     * Called when lantern[up] is on
     */
    private void lanternUp() {	    	
    	// set lanternUpOn
    	lanternUpOn = true;
    	// RT 8-1
    	if (!havePendingCall()) {
    		warning("Lantern is on when there are no pending call");
    	}
    	// RT 8-2
    	if (lanternDownOn) {
    		warning("Lantern[Down] is already on, passenger is confused");
    	}
    	// RT 8-3
    	if (!havePendingCallUp(currentFloor)) {
    		warning("Lantern[Up] is on when no more pending call above this floor.");
    	}
    }
    /**
     * Called when lantern[down] is on
     */
    private void lanternDown() {    	
    	// set lanternDownOn
    	lanternDownOn = true;
    	// RT 8-1
    	if (!havePendingCall()) {
    		warning("Lantern is on when there are no pending call");
    	}
    	// RT 8-2
    	if (lanternUpOn) {
    		warning("Lantern[Up] is already on, passenger is confused");
    	}
    	// RT 8-3
    	if (!havePendingCallDown(currentFloor)) {
    		warning("Lantern[Down] is on when no more pending call below this floor.");
    	}
    }
    /**
     * Called when both lanterns are on
     */
    private void lanternBoth() {
    	warning("Both lanterns are on at same time");
    }
    
    /*************************************************************************
     * Qiang utility
     *************************************************************************/
    private boolean havePendingCall() {
    	for (int i = 0; i < 8; i++) {
    		if (HallCall[i][0][0] || HallCall[i][0][1] || HallCall[i][1][0] || HallCall[i][1][1]) {
    			return true;
    		}
    		if (CarCall[i][0] || CarCall[i][1]) {
    			return true;
    		}
    	}
    	return false;
    }
    private boolean havePendingCallUp(int cf) {
    	for (int i = cf; i < 8; i++) {
    		if (HallCall[i][0][0] || HallCall[i][0][1] || HallCall[i][1][0] || HallCall[i][1][1]) {
    			return true;
    		}
    		if (CarCall[i][0] || CarCall[i][1]) {
    			return true;
    		}
    	}
    	return false;
    }
    private boolean havePendingCallDown(int cf) {
    	for (int i = cf - 2; i > -1; i--) {
    		if (HallCall[i][0][0] || HallCall[i][0][1] || HallCall[i][1][0] || HallCall[i][1][1]) {
    			return true;
    		}
    		if (CarCall[i][0] || CarCall[i][1]) {
    			return true;
    		}
    	}
    	return false;
    }
    private boolean haveCall(int floor) {
    	int floorIndex = floor -1;
    	if (HallCall[floorIndex][0][0] || HallCall[floorIndex][0][1] || HallCall[floorIndex][1][0] || HallCall[floorIndex][1][1]) {
    		return true;
    	} else if (CarCall[floorIndex][0] || CarCall[floorIndex][1]) {
    		return true;
    	}
    	return false;
    }
    
    /**************************************************************************
     * low level message receiving methods
     * 
     * These mostly forward messages to the appropriate state machines
     **************************************************************************/
    @Override
    public void receive(ReadableDoorClosedPayload msg) {
        doorState.receive(msg);
    }
    @Override
    public void receive(ReadableDoorOpenPayload msg) {
        doorState.receive(msg);
    }
    @Override
    public void receive(ReadableDoorMotorPayload msg) {
        doorState.receive(msg);
    }
    @Override
    public void receive(ReadableCarWeightPayload msg) {
        weightState.receive(msg);
    }
    @Override
    public void receive(ReadableHallCallPayload msg) {
    	hallcallState.receive(msg);
    }
    @Override
    public void receive(ReadableCarCallPayload msg) {
    	carcallState.receive(msg);
    }
    @Override
    public void receive(ReadableDrivePayload msg) {
    	doorState.receive(msg);
    	driveState.receive(msg);
    }
    @Override
    public void receive(ReadableAtFloorPayload msg) {
    	doorState.receive(msg);
    	if (msg.getValue()) {
    		currentFloor = msg.getFloor();
    	}
    }
    @Override
    public void receive(ReadableDoorReversalPayload msg) {
    	reversalState.receive(msg);
    	int side = msg.getSide().ordinal();
    	int hallway = msg.getHallway().ordinal();
    	tmpReversalCount[hallway][side] += 1;
    }
    @Override
    public void receive(ReadableDriveSpeedPayload msg) {
        if (msg.speed() > 0) {
            hasMoved = true;
        }
    }
    @Override
    public void receive(ReadableCarLanternPayload msg) {
    	lanternState.receive(msg);
    }
    /*************************************************************************
     * States
     *************************************************************************/
    private static enum DoorState {
        CLOSED,
        OPENING,
        OPEN,
        CLOSING,
        NUDGING
    }
    private static enum LanternState {
    	Lantern_Off,
    	Lantern_Up_On,
    	Lantern_Down_On,
    	Lantern_Both_On
    }
    private static enum DriveState {
    	STOP,
    	SLOW,
    	FAST
    }
    
    /**
     * Utility class to detect hall call
     */
    private class HallCallStateMachine {
    	public void receive(ReadableHallCallPayload msg) {
    		Hallway hallway = msg.getHallway();
    		int hallIndex = hallway.ordinal();
    		Direction dir = msg.getDirection();
    		int dirIndex = dir.ordinal();
    		int floor = msg.getFloor();
    		if (msg.pressed()) {
    			HallCall[floor - 1][hallIndex][dirIndex] = true;
    		}
    	}
    }
    
    /**
     * Utility class to detect car call
     */
    private class CarCallStateMachine {
    	public void receive(ReadableCarCallPayload msg) {
    		Hallway hallway = msg.getHallway();
    		int hallIndex = hallway.ordinal();
    		int floor = msg.getFloor();
    		if (msg.pressed()) {
    			CarCall[floor - 1][hallIndex] = true;
    		}
    	}
    }
    /**
     * Utility class to detect reversal
     */
    private class ReversalStateMachine {
    	public void receive(ReadableDoorReversalPayload msg) {
    		if (msg.isReversing()) {
    			isReversal = true;
    		}
    	}
    }
    
    /**
     * Utility class to detect weight changes
     */
    private class WeightStateMachine {
        int oldWeight = 0;
        public void receive(ReadableCarWeightPayload msg) {
            if (oldWeight != msg.weight()) {
                weightChanged(msg.weight());
            }
            oldWeight = msg.weight();
        }
    }
    
    /**
     * Qiang Code
     * Utility class for keeping track of the drive state
     */
    private class DriveStateMachine {
    	private DriveState state;
    	public DriveStateMachine() {
    		state = DriveState.STOP;
    	}
    	public void receive(ReadableDrivePayload msg) {
    		updateState();
    	}
    	private void updateState() {
    		DriveState previousState = state;
    		DriveState newState = previousState;
    		if (driveCommandedSpeed.speed() == Speed.STOP) {
    			newState = DriveState.STOP;
    		} else if (driveCommandedSpeed.speed() == Speed.SLOW) {
    			newState = DriveState.SLOW;
    		} else if (driveCommandedSpeed.speed() == Speed.FAST) {
    			newState = DriveState.FAST;
    		}
    		if (previousState != newState) {
    			switch(newState) {
    				case STOP:
    					driveStop();
    					break;
    				case SLOW:
    					driveSlow();
    					break;
    				case FAST:
    					driveFast();
    					break;
    			}
    		}
    		// keep checking moving speed
    		if (newState == DriveState.SLOW) {
    			driveSlow();
    		}
    		state = newState;
    	}
    }
    
    /**
     * Qiang Code
     * Utility class for keeping track of the lantern state
     */
    private class LanternStateMachine {
    	private LanternState state;
    	
    	public LanternStateMachine() {
    		state = LanternState.Lantern_Off;
    	}
    	public void receive(ReadableCarLanternPayload msg) {
    		updateState();
    	}
    	private void updateState() {
    		LanternState previousState = state;
    		LanternState newState = previousState;
    		if (allLanternOff()) {
    			newState = LanternState.Lantern_Off;
    		} else if (carLanterns[0].lighted() && (!carLanterns[1].lighted())) {
    			newState = LanternState.Lantern_Up_On;
    		} else if ((!carLanterns[0].lighted()) && carLanterns[1].lighted()) {
    			newState = LanternState.Lantern_Down_On;
    		} else {
    			newState = LanternState.Lantern_Both_On;
    		}
    		if (previousState != newState) {
    			switch(newState) {
    				case Lantern_Off:
    					lanternOff();
    					break;
    				case Lantern_Up_On:
    					lanternUp();
    					break;
    				case Lantern_Down_On:
    					lanternDown();
    					break;
    				case Lantern_Both_On:
    					lanternBoth();
    					break;
    				default:
    					break;
    			}
    		}
    		// set new State
    		state = newState;
    	}
    	private boolean allLanternOff() {
    		return (!carLanterns[0].lighted()) & (!carLanterns[1].lighted());
    	}
    }

    /**
     * Utility class for keeping track of the door state.
     * 
     * Also provides external methods that can be queried to determine the
     * current door state.
     */
    private class DoorStateMachine {
        DoorState state[] = new DoorState[2];
        int curFloor = -1;
        Direction targetDir = Direction.STOP;
        Speed curSpeed = Speed.SLOW;
        public DoorStateMachine() {
            state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
            state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
        }
        public void receive(ReadableDoorClosedPayload msg) {
            updateState(msg.getHallway());
        }
        public void receive(ReadableDoorOpenPayload msg) {
            updateState(msg.getHallway());
        }
        public void receive(ReadableDoorMotorPayload msg) {
            updateState(msg.getHallway());
        }
        public void receive(ReadableAtFloorPayload msg) {
        	if (msg.getValue()) {
        		curFloor = msg.getFloor();
        	}
    	}
    	public void receive(ReadableDrivePayload msg) {
    		targetDir = msg.direction();
    		curSpeed = msg.speed();
    	}
        private void updateState(Hallway h) {
            DoorState previousState = state[h.ordinal()];
            DoorState newState = previousState;
            if (allDoorsClosed(h) && allDoorMotorsStopped(h)) {
                newState = DoorState.CLOSED;
            } else if (allDoorsCompletelyOpen(h) && allDoorMotorsStopped(h)) {
                newState = DoorState.OPEN;
            } else if (anyDoorMotorClosing(h)) {
                newState = DoorState.CLOSING;
            } else if (anyDoorMotorOpening(h)) {
                newState = DoorState.OPENING;
            } else if (anyDoorMotorNudging(h)) {
            	newState = DoorState.NUDGING;
            }
            if (newState != previousState) {
                switch (newState) {
                    case CLOSED:
                    	// qiang code
                    	int floorIndex = curFloor - 1;
                    	int dirindex = -1;
                    	int hallwayIndex = (h.equals(Hallway.FRONT)) ? 0 : 1;
                    	if (targetDir.equals(Direction.UP)) {
                    		dirindex = 0;
                    	} else if (targetDir.equals(Direction.DOWN)) {
                    		dirindex = 1;
                    	}
                    	if (dirindex >= 0 && curSpeed.equals(Speed.STOP)) {
                    		// if there is no hall call at hallway set the noHallCall to true
                    		if (HallCall[floorIndex][hallwayIndex][0] == false && HallCall[floorIndex][hallwayIndex][1] == false) {
                    			noHallCall[hallwayIndex] = true;
                    		}
                    		// if there is no car call at the floor of the hallway, set the noCarCall to true
                    		if (CarCall[floorIndex][hallwayIndex] == false) {
                    			noCarCall[hallwayIndex] = true;
                    		}
                    	}
                    	// qiang code end
                        doorClosed(h);
                        break;
                    case OPEN:
                        doorOpened(h);
                        break;
                    case OPENING:
                        if (previousState == DoorState.CLOSING) {
                            doorReopening(h);
                        } else {
                            doorOpening(h);
                        }
                        break;
                    case CLOSING:
                        doorClosing(h);
                        break;
                    case NUDGING:
                    	doorNudging(h);
                    	break;
                }
            }
            //set the newState
            state[h.ordinal()] = newState;
        }
        //door utility methods
        public boolean allDoorsCompletelyOpen(Hallway h) {
            return doorOpeneds[h.ordinal()][Side.LEFT.ordinal()].isOpen()
                    && doorOpeneds[h.ordinal()][Side.RIGHT.ordinal()].isOpen();
        }
        public boolean anyDoorOpen() {
            return anyDoorOpen(Hallway.FRONT) || anyDoorOpen(Hallway.BACK);
        }
        public boolean anyDoorOpen(Hallway h) {
            return !doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
                    || !doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed();
        }
        public boolean allDoorsClosed(Hallway h) {
            return (doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
                    && doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed());
        }
        public boolean allDoorMotorsStopped(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.STOP && doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.STOP;
        }
        public boolean anyDoorMotorOpening(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.OPEN || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.OPEN;
        }
        public boolean anyDoorMotorClosing(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.CLOSE || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.CLOSE;
        }
        public boolean anyDoorMotorNudging(Hallway h) {
        	return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.NUDGE || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.NUDGE;
        }
    }

    /**
     * Keep track of time and decide whether to or not to include the last interval
     */
    private class ConditionalStopwatch {
        private boolean isRunning = false;
        private SimTime startTime = null;
        private SimTime accumulatedTime = SimTime.ZERO;
        /**
         * Call to start the stopwatch
         */
        public void start() {
            if (!isRunning) {
                startTime = Harness.getTime();
                isRunning = true;
            }
        }
        /**
         * stop the stopwatch and add the last interval to the accumulated total
         */
        public void commit() {
            if (isRunning) {
                SimTime offset = SimTime.subtract(Harness.getTime(), startTime);
                accumulatedTime = SimTime.add(accumulatedTime, offset);
                startTime = null;
                isRunning = false;
            }
        }
        /**
         * stop the stopwatch and discard the last interval
         */
        public void reset() {
            if (isRunning) {
                startTime = null;
                isRunning = false;
            }
        }
        public SimTime getAccumulatedTime() {
            return accumulatedTime;
        }
        public boolean isRunning() {
            return isRunning;
        }
    }

    /**
     * Keep track of the accumulated time for an event
     */
    private class Stopwatch {
        private boolean isRunning = false;
        private SimTime startTime = null;
        private SimTime accumulatedTime = SimTime.ZERO;
        /**
         * Start the stopwatch
         */
        public void start() {
            if (!isRunning) {
                startTime = Harness.getTime();
                isRunning = true;
            }
        }
        /**
         * Stop the stopwatch and add the interval to the accumulated total
         */
        public void stop() {
            if (isRunning) {
                SimTime offset = SimTime.subtract(Harness.getTime(), startTime);
                accumulatedTime = SimTime.add(accumulatedTime, offset);
                startTime = null;
                isRunning = false;
            }
        }
        public SimTime getAccumulatedTime() {
            return accumulatedTime;
        }
        public boolean isRunning() {
            return isRunning;
        }
    }

    /**
     * Utility class to implement an event detector
     */
    private abstract class EventDetector {

        boolean previousState;

        public EventDetector(boolean initialValue) {
            previousState = initialValue;
        }

        public void updateState(boolean currentState) {
            if (currentState != previousState) {
                previousState = currentState;
                eventOccurred(currentState);
            }
        }

        /**
         * subclasses should overload this to make something happen when the event
         * occurs.
         * @param newState
         */
        public abstract void eventOccurred(boolean newState);
    }
}
