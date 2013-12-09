/**
* 18649 Fall 2013
* Group 17
* Qiang Zhang(qiangz), Shen Yu(sheny), Jiang He(jiangh), Xianle Wang(xianlew)
**/
package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DesiredDwellCanPayloadTranslator extends CanPayloadTranslator {
	
	/**
     * Constructor for WriteableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DesiredDwellCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 2, MessageDictionary.DESIRED_DWELL_BASE_CAN_ID);
    }

    /**
     * Constructor for ReadableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DesiredDwellCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 2, MessageDictionary.DESIRED_DWELL_BASE_CAN_ID);
    }
    
    
    public void set(int value) {
        setValue(value);
    }
    

    public void setValue(int value) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, value, 0, 16);
        setMessagePayload(b, getByteSize());
    }
    
    public int getValue() {
        return getIntFromBitset(getMessagePayload(), 0, 16);
    }
    
    public String payloadToString() {
        return "DesiredDwell = " + getValue();
    }
	
	
}
