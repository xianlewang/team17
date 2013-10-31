/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.payloads.CANNetwork;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/**
 * This class provides some example utility classes that might be useful in more
 * than one spot.  It is okay to create new classes (or modify the ones given
 * below), but you may not use utility classes in such a way that they constitute
 * a communication channel between controllers.
 *
 * @author justinr2
 */
public class Utility {

    public static class DoorClosedArray {

        HashMap<Integer, DoorClosedCanPayloadTranslator> translatorArray = new HashMap<Integer, DoorClosedCanPayloadTranslator>();
        public final Hallway hallway;

        public DoorClosedArray(Hallway hallway, CANNetwork.CanConnection conn) {
            this.hallway = hallway;
            for (Side s : Side.values()) {
                int index = ReplicationComputer.computeReplicationId(hallway, s);
                ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + index);
                DoorClosedCanPayloadTranslator t = new DoorClosedCanPayloadTranslator(m, hallway, s);
                conn.registerTimeTriggered(m);
                translatorArray.put(index, t);
            }
        }

        public boolean getBothClosed() {
            return translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.LEFT)).getValue() &&
                    translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.RIGHT)).getValue();
        }
    }
    public static class NearestFloor {
    	private ReadableCanMailbox networkCarLevelPosition;
    	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    	
    	public NearestFloor(CANNetwork.CanConnection conn) {
    		networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
    		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
    		conn.registerTimeTriggered(networkCarLevelPosition);
    	}
    	
    	public int getvalue() {
    		int value = 0;
    		if (mCarLevelPosition.getPosition() <= 2500)
    			value = 1;
    		else if (mCarLevelPosition.getPosition() > 2500 && mCarLevelPosition.getPosition() <= 7500)
    			value = 2;
    		else if (mCarLevelPosition.getPosition() > 7500 && mCarLevelPosition.getPosition() <= 12500)
    			value = 3;
    		else if (mCarLevelPosition.getPosition() > 12500 && mCarLevelPosition.getPosition() <= 17500)
    			value = 4;
    		else if (mCarLevelPosition.getPosition() > 17500 && mCarLevelPosition.getPosition() <= 22500)
    			value = 5;
    		else if (mCarLevelPosition.getPosition() > 22500 && mCarLevelPosition.getPosition() <= 27500)
    			value = 6;
    		else if (mCarLevelPosition.getPosition() > 27500 && mCarLevelPosition.getPosition() <= 32500)
    			value = 7;
    		else if (mCarLevelPosition.getPosition() > 32500)
    			value = 8;
    		return value;
    	}
    	
    	public int getLevelPosition() {
    		return mCarLevelPosition.getPosition();
    	}
    }
    public static class AtFloorArray {

        public HashMap<Integer, AtFloorCanPayloadTranslator> networkAtFloorsTranslators = new HashMap<Integer, AtFloorCanPayloadTranslator>();
        public final int numFloors = Elevator.numFloors;

        public AtFloorArray(CANNetwork.CanConnection conn) {
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
                    AtFloorCanPayloadTranslator t = new AtFloorCanPayloadTranslator(m, floor, h);
                    conn.registerTimeTriggered(m);
                    networkAtFloorsTranslators.put(index, t);
                }
            }
        }
        
        public boolean isAtFloor(int floor, Hallway hallway) {
            return networkAtFloorsTranslators.get(ReplicationComputer.computeReplicationId(floor, hallway)).getValue();
        }

        public int getCurrentFloor() {
            int retval = MessageDictionary.NONE;
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    AtFloorCanPayloadTranslator t = networkAtFloorsTranslators.get(index);
                    if (t.getValue()) {
                        if (retval == MessageDictionary.NONE) {
                            //this is the first true atFloor
                            retval = floor;
                        } else if (retval != floor) {
                            //found a second floor that is different from the first one
                            throw new RuntimeException("AtFloor is true for more than one floor at " + Harness.getTime());
                        }
                    }
                }
            }
            return retval;
        }
    }
    
    public static class CallArray {

        HashMap<Integer, BooleanCanPayloadTranslator> carCallArray = new HashMap<Integer, BooleanCanPayloadTranslator>();
        HashMap<Integer, BooleanCanPayloadTranslator> hallCallArray = new HashMap<Integer, BooleanCanPayloadTranslator>();
        List<Integer> carIndexList = new ArrayList<Integer>();
        List<Integer> hallIndexList = new ArrayList<Integer>();

        public CallArray(CANNetwork.CanConnection conn) {
            // ini car call
            for (Hallway b : Hallway.replicationValues) {

                for (int f = 1; f <= 8; ++f) {
                    int index = ReplicationComputer.computeReplicationId(f, b);
                    carIndexList.add(index);
                    ReadableCanMailbox m = CanMailbox
                            .getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID
                                    + index);
                    BooleanCanPayloadTranslator t = new BooleanCanPayloadTranslator(
                            m);
                    conn.registerTimeTriggered(m);
                    carCallArray.put(index, t);
                }
            }

            // ini hall call
            for (Hallway b : Hallway.replicationValues) {
                for (Direction d : Direction.replicationValues) {
                    for (int f = 1; f <= 8; ++f) {
                        int index = ReplicationComputer.computeReplicationId(f,
                                b, d);
                        hallIndexList.add(index);

                        ReadableCanMailbox m = CanMailbox
                                .getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID
                                        + index);
                        BooleanCanPayloadTranslator t = new BooleanCanPayloadTranslator(
                                m);
                        conn.registerTimeTriggered(m);
                        hallCallArray.put(index, t);
                    }
                }
            }
        }

        public int size() {
            int sum = 0;
            for (int i : carIndexList) {
                if (carCallArray.get(i).getValue()) {
                    ++sum;
                }
            }
            for (int i : hallIndexList) {
                if (hallCallArray.get(i).getValue()) {
                    ++sum;
                }
            }
            return sum;
        }

        public boolean isCalled(int f, Direction d) {
            //System.out.println("is called? "+f+d);
            int iCarFront = ReplicationComputer.computeReplicationId(f,
                    Hallway.FRONT);
            int iCarBack = ReplicationComputer.computeReplicationId(f,
                    Hallway.BACK);
            if (carCallArray.get(iCarFront).getValue()
                    || carCallArray.get(iCarBack).getValue()) {
                return true;
            }
            int iHallFront = ReplicationComputer.computeReplicationId(f,
                    Hallway.FRONT, d);
            int iHallBack = ReplicationComputer.computeReplicationId(f,
                    Hallway.BACK, d);
            if (hallCallArray.get(iHallFront).getValue()
                    || hallCallArray.get(iHallBack).getValue()) {
                return true;
            }
            return false;
        }
        public boolean isCalled(int f, Direction d, Hallway b) {
            int iCar = ReplicationComputer.computeReplicationId(f,
                    b);

            if (carCallArray.get(iCar).getValue()) {
                return true;
            }
            
            int iHall = ReplicationComputer.computeReplicationId(f,
                    b, d);
            if (hallCallArray.get(iHall).getValue()) {
                return true;
            }
            return false;
        }

        public Hallway getHallway(int f, Direction d) {
            int front = 0;
            int back = 0;
            int iCarFront = ReplicationComputer.computeReplicationId(f,
                    Hallway.FRONT);
            int iCarBack = ReplicationComputer.computeReplicationId(f,
                    Hallway.BACK);
            if (carCallArray.get(iCarFront).getValue())
                front = 1;
            if (carCallArray.get(iCarBack).getValue())
                back = 1;

            int iHallFront = ReplicationComputer.computeReplicationId(f,
                    Hallway.FRONT, d);
            int iHallBack = ReplicationComputer.computeReplicationId(f,
                    Hallway.BACK, d);
            if (hallCallArray.get(iHallFront).getValue())
                front = 1;
            if (hallCallArray.get(iHallBack).getValue())
                back = 1;
            
            if(front==1&&back==1)
                return Hallway.BOTH;
            else if(front==1){
                return Hallway.FRONT;
            }else if(back==1){
                return Hallway.BACK;
            }
            return Hallway.NONE;
        }
    }
}
