/*
 18649 Fall 2013
 Group 17
 Qiang Zhang(qiangz)
 Shen Yu (sheny), Jiang He (jiangh), Xianle Wang (xianlew)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
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
import simulator.payloads.CarWeightPayload.ReadableCarWeightPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.DrivePayload.ReadableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;


public class Proj7RuntimeMonitor extends RuntimeMonitor {

    DoorStateMachine doorState = new DoorStateMachine();
    WeightStateMachine weightState = new WeightStateMachine();
    HallCallStateMachine hallcallState = new HallCallStateMachine();
    CarCallStateMachine  carcallState = new CarCallStateMachine();
    ReversalStateMachine reversalState = new ReversalStateMachine();
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
    int ReversalCount = 0;
    public Proj7RuntimeMonitor() {
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
    }

    @Override
    protected String[] summarize() {
        String[] arr = new String[3];
        arr[0] = "Overweight Count = " + overWeightCount;
        arr[1] = "NoCall Count = " + noCallCount;
        arr[2] = "Reversal Count = " + ReversalCount;  
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
    		message("No Call At Front");
    		noCallCount += 1;
    		noHallCall[0] = false;
    		noCarCall[0]  = false;
    	}
    	if (noHallCall[1] && noCarCall[1]) {
    		message("No Hall Call At Back");
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
    		ReversalCount += 1;
    		message("Reverse");
    		isReversal = false;
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
    }
    @Override
    public void receive(ReadableAtFloorPayload msg) {
    	doorState.receive(msg);
    }
    @Override
    public void receive(ReadableDoorReversalPayload msg) {
    	reversalState.receive(msg);
    }
    @Override
    public void receive(ReadableDriveSpeedPayload msg) {
        if (msg.speed() > 0) {
            hasMoved = true;
        }
    }
    private static enum DoorState {
        CLOSED,
        OPENING,
        OPEN,
        CLOSING
    }
    
    /**
     * Utility class to detect hall call
     */
    private class HallCallStateMachine {
    	public void receive(ReadableHallCallPayload msg) {
    		Hallway hallway = msg.getHallway();
    		int hallIndex = (hallway.equals(Hallway.FRONT)) ? 0 : 1;
    		Direction dir = msg.getDirection();
    		int dirIndex = (dir.equals(Direction.UP)) ? 0 : 1;
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
    		int hallIndex = (hallway.equals(Hallway.FRONT)) ? 0 : 1;
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
