/*
 18649 Fall 2013
 Group 17
 Jiang He(jiangh)
 (other names would go here)
 */
package simulator.elevatorcontrol;

import java.util.BitSet;
import simulator.framework.Direction;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * Can payload translator for the drive speed, which includes a speed 
 * value(numeric) and a direction.
 * 
 * See the documentation for DesiredFloorCanPayloadTranslator for more discussion
 * on CanPayloadTranslators in general.
 * 
 * @author Jiang He
 */
public class DriveSpeedCanPayloadTranslator extends CanPayloadTranslator {

    public DriveSpeedCanPayloadTranslator(WriteableCanMailbox p) {
        super(p, 8, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }
    
    public DriveSpeedCanPayloadTranslator(ReadableCanMailbox p) {
        super(p, 8, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }

    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param speed
     * @param dir
     */
    public void set(double speed, Direction dir) {
        setSpeed(speed);
        setDirection(dir);
    }
    
    public void setSpeed(double speed) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, (int)(speed*1000), 0, 32);
        setMessagePayload(b, getByteSize());
    }

    public double getSpeed() {
        int val = getIntFromBitset(getMessagePayload(), 0, 32);
         
		  return val/1000;
    }

    public void setDirection(Direction dir) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, dir.ordinal(), 32, 32);
        setMessagePayload(b, getByteSize());
    }

    public Direction getDirection() {
        int val = getIntFromBitset(getMessagePayload(), 32, 32);
        for (Direction d : Direction.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }

    @Override
    public String payloadToString() {
        return "DriveCommand:  speed=" + getSpeed() + " direction=" + getDirection();
    }
}
