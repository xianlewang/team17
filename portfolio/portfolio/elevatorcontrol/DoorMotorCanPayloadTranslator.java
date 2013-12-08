/*
 18649 Fall 2013
 Group 17
 Qiang Zhang(qiangz)
 (other names would go here)
 */
package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DoorMotorCanPayloadTranslator extends CanPayloadTranslator{

	public DoorMotorCanPayloadTranslator(ReadableCanMailbox payload, Hallway hallway, Side side) {
		super(payload, 1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
	}
	
	public DoorMotorCanPayloadTranslator(WriteableCanMailbox payload, Hallway hallway, Side side) {
		super(payload, 1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
	}
	public void set(DoorCommand doorC) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, doorC.ordinal(), 0, 8);
        setMessagePayload(b, getByteSize());
	}
	public void setDoorCommand(DoorCommand doorC) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, doorC.ordinal(), 0, 8);
        setMessagePayload(b, getByteSize());
	}
	
	public DoorCommand getDoorCommand() {
        int val = getIntFromBitset(getMessagePayload(), 0, 8);
        for (DoorCommand doorC : DoorCommand.values()) {
            if (doorC.ordinal() == val) {
                return doorC;
            }
        }
        throw new RuntimeException("Unrecognized DoorCommand Value " + val);
	}
	@Override
	public String payloadToString() {
        return "DoorMotor:  command=" + getDoorCommand();
	}
		
}
