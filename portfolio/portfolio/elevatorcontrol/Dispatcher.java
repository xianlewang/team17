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
    // private int target = 1;
    // private Hallway desiredHallway = null;
    private Target target;
    private AtFloorArray atFloorArray = null;
    private int dwell = 1000;
    private int wait = 0;
    // private Direction desiredDir = Direction.STOP;
    // private Direction movedDir = Direction.STOP;
    private CallArray callArray;

    // enumerate states
    private enum State {
        OPERATING, EMERGENCY, IDLE, REACH, LEAVE
    }

    // state variable initialized to the initial state FLASH_OFF
    private State state = State.IDLE;
    private Direction lanternDir = Direction.STOP;
    private Direction movingDir = Direction.STOP;
    

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
        target = new Target(1, Direction.STOP, Hallway.NONE);
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
    @Override
    public void timerExpired(Object callbackData) {
        State newState = state;
        switch (state) {
        case IDLE:
            //System.out.println("in IDLE");
            // state var
            wait=0;
            target = new Target(currentFloor, Direction.STOP, Hallway.NONE);
            currentFloor = atFloorArray.getCurrentFloor();
            boolean doorAllClosed = (doorClosedBack.getBothClosed() && doorClosedFront.getBothClosed());
            // state actions for 'HOLD'
            mDesiredFloor.set(currentFloor, Direction.STOP, Hallway.NONE);
            mDesiredDwell_b.set(dwell);
            mDesiredDwell_f.set(dwell);
            // System.out.println("---callArray.size= "+callArray.size());
// #transition 'T11.3'
            if (atFloorArray.getCurrentFloor() == MessageDictionary.NONE
                    && !doorAllClosed) {
                newState = State.EMERGENCY;
            } else if (callArray.size() > 0) {
// #transition 'T11.1'
                newState = State.REACH;
                lanternDir = Direction.STOP;
            }
            break;
        case OPERATING:
            //System.out.println("in OPERATING");
            // state var
            wait=0;
            currentFloor = atFloorArray.getCurrentFloor();
            target = getTarget(movingDir);
            doorAllClosed = (doorClosedBack.getBothClosed() && doorClosedFront.getBothClosed());
            // state actions for 'OPERATING'
            mDesiredFloor.set(target.f, target.d, target.b);
// #transition 'T11.3'
            if (atFloorArray.getCurrentFloor() == MessageDictionary.NONE
                    && ! doorAllClosed ) {
                newState = State.EMERGENCY;
            } else if (! doorAllClosed && currentFloor == target.f) {
// #transition 'T11.4'
                newState = State.REACH;
                lanternDir  = target.d;
            }
            break;
        case EMERGENCY:
            target = new Target(1, Direction.STOP, Hallway.NONE);
            mDesiredFloor.set(1, Direction.STOP, Hallway.NONE);
            break;
        case REACH:
            // state var
            currentFloor = atFloorArray.getCurrentFloor();
            target = getTarget2(lanternDir);
            doorAllClosed = (doorClosedBack.getBothClosed() && doorClosedFront.getBothClosed());
            if(! doorAllClosed){
                wait = 0;
            }
            // state actions for 'OPERATING'
            mDesiredFloor.set(target.f, target.d, target.b);
            //System.out.println("in REACH, wait = "+wait+" target"+target.f+" doorClosed: "+ doorAllClosed);
// #transition 'T11.3'
            if (atFloorArray.getCurrentFloor() == MessageDictionary.NONE
                    && !(doorClosedBack.getBothClosed() && doorClosedFront
                            .getBothClosed())) {
                newState = State.EMERGENCY;
            } else if ((target.f == currentFloor &&doorAllClosed&& wait>5)||(callArray.size()==0&&lanternDir == Direction.STOP)) {
// #transition 'T11.2'
                //System.out.println("to IDLE!-------------------------------------wait = "+wait);
                wait = 0;
                newState = State.IDLE;
            } else if (mDriveSpeed.getSpeed()>0.05||(target.f!=currentFloor&& doorAllClosed)) {
// #transition 'T11.6'
                //System.out.println("REACH-->LEAVE");
                newState = State.LEAVE;
            }
            break;
        case LEAVE:
            //System.out.println("in LEAVE");
            // state var
            doorAllClosed = (doorClosedBack.getBothClosed() && doorClosedFront.getBothClosed());
            currentFloor = atFloorArray.getCurrentFloor();
            mDesiredFloor.set(target.f, target.d, target.b);
            if (atFloorArray.getCurrentFloor() == MessageDictionary.NONE
                    && !doorAllClosed) {
// #transition 'T11.3'
                newState = State.EMERGENCY;
            } else if (mCarLevelPosition.getPosition()%5000>200&&mCarLevelPosition.getPosition()%5000<4800) {
// #transition 'T11.5'
                movingDir = mDriveSpeed.getDirection();
                newState = State.OPERATING;
            }
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
    //called when moving or reaching
    private Target getTarget(Direction tmpDir) {
        List<Integer> floors = getCommitFloorList(tmpDir);
        //System.out.println(tmpDir+"/"+target.f+"+"+target.d);
        // if (mDriveSpeed.getSpeed() < 0.3) {
        //     if ((mDriveSpeed.getDirection() == Direction.UP && mCarLevelPosition
        //             .getPosition() % 5000 < 200)
        //             || (mDriveSpeed.getDirection() == Direction.DOWN && mCarLevelPosition
        //                     .getPosition() % 5000 > 4800)) {
        //         int i = floors.indexOf(currentFloor);
        //         if (i != -1){
        //             floors.remove(i);
        //             //System.out.println("--------------------------------i="+i);
        //         }
        //     }
        // }


        //when from IDLE to REACH /when reach at car call
        if (tmpDir.equals(Direction.STOP)) {
            if (callArray.isCalled(currentFloor, Direction.STOP)) {
                return target;
            }
            for (int i : floors) {// 1-8 floor
                if (callArray.isCalled(i, Direction.STOP)) {
                    return new Target(i, Direction.STOP, callArray.getHallway(
                            i, Direction.STOP));
                }
                if (callArray.isCalled(i, Direction.UP)) {
                    return new Target(i, Direction.UP, callArray.getHallway(i,
                            Direction.UP));
                } else if (callArray.isCalled(i, Direction.DOWN)) {
                    return new Target(i, Direction.DOWN, callArray.getHallway(
                            i, Direction.DOWN));
                }
            }
            //System.out.println("!!!!!!!!");
            return target;
        }

        // looking up
        if (tmpDir == Direction.UP) {
            int carUp = 9;
            int sameUp = 9;
            int revUp = 0;
            for (int i : floors) {
                // find nearest car call
                if (callArray.isCalled(i, Direction.STOP)) {
                    if (carUp > i) {
                        carUp = i;
                    }
                }
                // find nearest sameDir Hall call
                if (callArray.isCalled(i, tmpDir)) {// choose the nearest
                    if (sameUp > i) {
                        sameUp = i;
                    }
                }

            }// end
            if (Math.min(carUp, sameUp) < 9) {
                if (carUp < sameUp) {
                    boolean flag1 = false;//has more calls ahead
                    for (int i = carUp + 1; i < 9; ++i) {
                        if (callArray.isCalled(i, Direction.UP)
                                || callArray.isCalled(i, Direction.DOWN)
                                || callArray.isCalled(i, Direction.STOP)) {
                            flag1 = true;
                            break;
                        }
                    }
                    boolean flag2 = false;//has more calls behind
                    for (int i = carUp - 1; i > 0; --i) {
                        if (callArray.isCalled(i, Direction.UP)
                                || callArray.isCalled(i, Direction.DOWN)
                                || callArray.isCalled(i, Direction.STOP)) {
                            flag2 = true;
                            break;
                        }
                    }
                    if (flag1) {
                        return new Target(carUp, Direction.UP,
                                callArray.getHallway(carUp, Direction.STOP));
                    } else {
                        if (callArray.isCalled(carUp, Direction.DOWN)|| flag2)  {
                            return new Target(carUp, Direction.DOWN,
                                    callArray.getHallway(carUp, Direction.DOWN));
                        } else {
                            return new Target(carUp, Direction.STOP,
                                    callArray.getHallway(carUp, Direction.STOP));
                        }
                    }
                } else if (sameUp <= carUp) {
                    return new Target(sameUp, Direction.UP,
                            callArray.getHallway(sameUp, Direction.UP));
                }
            }
            // find furtherest revDir hal call
            for (int i : floors) {
                if (callArray.isCalled(i, Direction.DOWN)) {// choose the
                                                            // furtherest
                    if (revUp < i&&i>currentFloor) {
                        revUp = i;
                    }
                }
            }
            if (revUp != 0) {
                return new Target(revUp, Direction.DOWN, callArray.getHallway(
                        revUp, Direction.DOWN));
            }
        }

        // looking down
        if (tmpDir == Direction.DOWN) {
            int carDown = 0;
            int sameDown = 0;
            for (int i : floors) {
                // find nearest car call
                if (callArray.isCalled(i, Direction.STOP)) {
                    if (carDown < i) {
                        carDown = i;
                    }
                }
                // find nearest sameDir Hall call
                if (callArray.isCalled(i, Direction.DOWN)) {// choose the
                                                            // nearest
                    if (sameDown < i) {
                        sameDown = i;
                    }
                }
            }// end
            if (Math.max(carDown, sameDown) > 0) {
                if (carDown > sameDown) {
                    boolean flag1 = false;
                    for (int i = carDown - 1; i > 0; --i) {
                        if (callArray.isCalled(i, Direction.UP)
                                || callArray.isCalled(i, Direction.DOWN)
                                || callArray.isCalled(i, Direction.STOP)) {
                            flag1 = true;
                            break;
                        }
                    }
                    boolean flag2 = false;
                    for (int i = carDown + 1; i < 9; ++i) {
                        if (callArray.isCalled(i, Direction.UP)
                                || callArray.isCalled(i, Direction.DOWN)
                                || callArray.isCalled(i, Direction.STOP)) {
                            flag2 = true;
                            break;
                        }
                    }
                    if (flag1) {
                        return new Target(carDown, Direction.DOWN,
                                callArray.getHallway(carDown, Direction.STOP));
                    } else {
                        if (callArray.isCalled(carDown, Direction.UP) || flag2) {
                            return new Target(carDown, Direction.UP,
                                    callArray.getHallway(carDown, Direction.UP));
                        } else {
                            return new Target(carDown, Direction.STOP,
                                    callArray.getHallway(carDown,
                                            Direction.STOP));
                        }
                    }
                } else if (sameDown >= carDown) {
                    return new Target(sameDown, Direction.DOWN,
                            callArray.getHallway(sameDown, Direction.DOWN));
                }
            }

            // no car call or sameDir hall call
            int revDown = 9;
            for (int i : floors) {
                if (callArray.isCalled(i, Direction.UP)) {// choose the
                                                            // furtherest
                    if (revDown > i&&i<currentFloor) {
                        revDown = i;
                    }
                }
            }// end
            if (revDown != 9) {
                return new Target(revDown, Direction.UP, callArray.getHallway(
                        revDown, Direction.UP));
            }
        }

        //wait++;
        return target;
    }

    private Target getTarget2(Direction tmpDir) {
        List<Integer> floors = getCommitFloorList(tmpDir);
        //System.out.println(tmpDir+"/"+target.f+"+"+target.d);
        // if (mDriveSpeed.getSpeed() < 0.3) {
        //     if ((mDriveSpeed.getDirection() == Direction.UP && mCarLevelPosition
        //             .getPosition() % 5000 < 200)
        //             || (mDriveSpeed.getDirection() == Direction.DOWN && mCarLevelPosition
        //                     .getPosition() % 5000 > 4800)) {
        //         int i = floors.indexOf(currentFloor);
        //         if (i != -1){
        //             floors.remove(i);
        //             //System.out.println("--------------------------------i="+i);
        //         }
        //     }
        // }


        //when from IDLE to REACH /when reach at car call
        if (tmpDir.equals(Direction.STOP)) {
            //System.out.println("currentFloor="+currentFloor+"mDriveSpeed"+mDriveSpeed.getSpeed()+" / "+mDriveSpeed.getDirection());
            if (callArray.isCalled(currentFloor, Direction.STOP)) {
                return target;
            }
            for (int i : floors) {// 1-8 floor
                if (callArray.isCalled(i, Direction.STOP)) {
                    return new Target(i, Direction.STOP, callArray.getHallway(
                            i, Direction.STOP));
                }
                if (callArray.isCalled(i, Direction.UP)) {
                    return new Target(i, Direction.UP, callArray.getHallway(i,
                            Direction.UP));
                } else if (callArray.isCalled(i, Direction.DOWN)) {
                    return new Target(i, Direction.DOWN, callArray.getHallway(
                            i, Direction.DOWN));
                }
            }
            //System.out.println("!!!!!!!!");
            return target;
        }

        // looking up
        if (tmpDir == Direction.UP) {
            int carUp = 9;
            int sameUp = 9;
            int revUp = 0;
            for (int i : floors) {
                // find nearest car call
                if (callArray.isCalled(i, Direction.STOP)) {
                    if (carUp > i) {
                        carUp = i;
                    }
                }
                // find nearest sameDir Hall call
                if (callArray.isCalled(i, tmpDir)) {// choose the nearest
                    if (sameUp > i) {
                        sameUp = i;
                    }
                }

            }// end
            if (Math.min(carUp, sameUp) < 9) {
                if (carUp < sameUp) {
                    boolean flag1 = false;//has more calls ahead
                    for (int i = carUp + 1; i < 9; ++i) {
                        if (callArray.isCalled(i, Direction.UP)
                                || callArray.isCalled(i, Direction.DOWN)
                                || callArray.isCalled(i, Direction.STOP)) {
                            flag1 = true;
                            break;
                        }
                    }
                    boolean flag2 = false;//has more calls behind
                    for (int i = carUp - 1; i > 0; --i) {
                        if (callArray.isCalled(i, Direction.UP)
                                || callArray.isCalled(i, Direction.DOWN)
                                || callArray.isCalled(i, Direction.STOP)) {
                            flag2 = true;
                            break;
                        }
                    }
                    if (flag1||carUp==currentFloor) {
                        return new Target(carUp, Direction.UP,
                                callArray.getHallway(carUp, Direction.STOP));
                    } else {
                        if (callArray.isCalled(carUp, Direction.DOWN)|| flag2)  {
                            return new Target(carUp, Direction.DOWN,
                                    callArray.getHallway(carUp, Direction.DOWN));
                        } else {
                            return new Target(carUp, Direction.STOP,
                                    callArray.getHallway(carUp, Direction.STOP));
                        }
                    }
                } else if (sameUp <= carUp) {
                    return new Target(sameUp, Direction.UP,
                            callArray.getHallway(sameUp, Direction.UP));
                }
            }
            // find furtherest revDir hal call
            for (int i : floors) {
                if (callArray.isCalled(i, Direction.DOWN)) {// choose the
                                                            // furtherest
                    if (revUp < i&&i>currentFloor) {
                        revUp = i;
                    }
                }
            }
            if (revUp != 0) {
                return new Target(revUp, Direction.DOWN, callArray.getHallway(
                        revUp, Direction.DOWN));
            }
        }

        // looking down
        if (tmpDir == Direction.DOWN) {
            int carDown = 0;
            int sameDown = 0;
            for (int i : floors) {
                // find nearest car call
                if (callArray.isCalled(i, Direction.STOP)) {
                    if (carDown < i) {
                        carDown = i;
                    }
                }
                // find nearest sameDir Hall call
                if (callArray.isCalled(i, Direction.DOWN)) {// choose the
                                                            // nearest
                    if (sameDown < i) {
                        sameDown = i;
                    }
                }
            }// end
            if (Math.max(carDown, sameDown) > 0) {
                if (carDown > sameDown) {
                    boolean flag1 = false;
                    for (int i = carDown - 1; i > 0; --i) {
                        if (callArray.isCalled(i, Direction.UP)
                                || callArray.isCalled(i, Direction.DOWN)
                                || callArray.isCalled(i, Direction.STOP)) {
                            flag1 = true;
                            break;
                        }
                    }
                    boolean flag2 = false;
                    for (int i = carDown + 1; i < 9; ++i) {
                        if (callArray.isCalled(i, Direction.UP)
                                || callArray.isCalled(i, Direction.DOWN)
                                || callArray.isCalled(i, Direction.STOP)) {
                            flag2 = true;
                            break;
                        }
                    }
                    if (flag1 || carDown==currentFloor) {
                        return new Target(carDown, Direction.DOWN,
                                callArray.getHallway(carDown, Direction.STOP));
                    } else {
                        if (callArray.isCalled(carDown, Direction.UP) || flag2) {
                            return new Target(carDown, Direction.UP,
                                    callArray.getHallway(carDown, Direction.UP));
                        } else {
                            return new Target(carDown, Direction.STOP,
                                    callArray.getHallway(carDown,
                                            Direction.STOP));
                        }
                    }
                } else if (sameDown >= carDown) {
                    return new Target(sameDown, Direction.DOWN,
                            callArray.getHallway(sameDown, Direction.DOWN));
                }
            }

            // no car call or sameDir hall call
            int revDown = 9;
            for (int i : floors) {
                if (callArray.isCalled(i, Direction.UP)) {// choose the
                                                            // furtherest
                    if (revDown > i&&i<currentFloor) {
                        revDown = i;
                    }
                }
            }// end
            if (revDown != 9) {
                return new Target(revDown, Direction.UP, callArray.getHallway(
                        revDown, Direction.UP));
            }
        }

        wait++;
        return target;
    }

    private Direction getTmpDir() {
        Direction tmpDir = Direction.STOP;
        // System.out.println("speed:----------------"+mDriveSpeed.getSpeed()+mDriveSpeed.getDirection());
        if (mDriveSpeed.getSpeed() > 0.06) {// while moving...
            tmpDir = mDriveSpeed.getDirection();
        } else {// stoped
            int pos = mCarLevelPosition.getPosition();
            if (target.f == currentFloor) {// target on current floor
                tmpDir = target.d;
            } else if ((target.f - 1) * 5000 > pos) {// target at upper floor
                tmpDir = Direction.UP;
            } else if ((target.f - 1) * 5000 < pos) {// target at upper floor
                tmpDir = Direction.DOWN;
            }
            // System.out.println("-----stoping...set dir "+tmpDir+"( moved: "+movedDir+" / desiredDir: "+target.d+")");
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
        dist = (int)(speed*speed*1000/2);
        //System.out.println("disp- : "+dist);
        // if (speed > 0.5) {
        //     dist = 500;
        // } else if (speed > 0.2) {
        //     dist = 50;
        // }
        if (tmpDir.equals(Direction.DOWN)) {
            int tmp = pos - dist;
            // System.out.println("commit point: " + tmp);

            for (int i = 0; (i * 5000 - 100 <= tmp) && i < 9; ++i) {
                out.add(i + 1);
            }
        } else if (tmpDir.equals(Direction.UP)) {
            int tmp = pos + dist;
            // System.out.println("commit point: " + tmp);

            for (int i = 7; (i * 5000 + 100 >= tmp) && i >= 0; --i) {
                out.add(i + 1);
            }
        } else {
            for (int i = 1; i <= 8; ++i) {
                out.add(i);
            }
        }
        Collections.sort(out);
        return out;
    }

    private static class Target {
        public int f;
        public Direction d;
        public Hallway b;

        public Target(int f, Direction d, Hallway b) {
            this.f = f;
            this.d = d;
            this.b = b;
        }
    }
}
