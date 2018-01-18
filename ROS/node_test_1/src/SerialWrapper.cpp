#include <vector>
#include <stdint.h>

class SerialWrapper {
	const uint8_t START_FLAG = 0x7E;
	const uint8_t STOP_FLAG = 0x7E;
	const uint8_t ESCAPE_FLAG = 0x7D;
	const uint8_t ESCAPE_MASK = 0x20;

	public:
		SerialWrapper() {
		
		}
		
	//Wraps serial message using the PPP serial encapsulation
	uint8_t* wrap(uint8_t byte_array[]) {
		//Create a vector with the start flag
		std::vector<uint8_t> temp_vect = {START_FLAG};
		
		//Iterate through the bytes in the given message
		int num_of_bytes = sizeof(byte_array)/sizeof(byte_array[0]);
		for (int i = 0; i < num_of_bytes; i++){
			switch (i) {
				/*
				 * If the data is the same as
				 * any flags we mask (escape) that data
				 */
				case START_FLAG:
				case STOP_FLAG:
				case ESCAPE_FLAG:
					temp_vect.push_back(ESCAPE_FLAG);
					temp_vect.push_back(escapeByte(byte_array[i]));
					break;
				//Otherwise add the data to the vector
				default:
					temp_vect.push_back(byte_array[i]);
					break;
			}
		}
		//Add the stop flag to the end of the message
		temp_vect.push_back(STOP_FLAG);
		
		//Return the new byte array
		return vect2ByteArray(temp_vect);
	}
	
	//Unwraps the serial message assuming PPP serial encapsulation
	uint8_t* unwrap(uint8_t byte_array[]){
		
		//Checks for the start flag
		if (byte_array[0] == START_FLAG){

			int i = 1;
			std::vector<uint8_t> new temp_vect = {};
			
			/* 
			 * If the start flag is found, iterate through 
			 * the message until the stop flag is found
			 */
			while (byte_array[i] != STOP_FLAG){
				uint8_t bite = byte_array[i];
				switch (bite){
					//Unmask data following escape flags and add to vector
					case ESCAPE_FLAG: temp_vect.push_back(escapeByte(bite)); break;
					//Otherwise just add to vector
					default: temp_vect.push_back(bite); break;
				}
			}
			//When the while loop is finished return the new byte array
			return vect2ByteArray(temp_vect);
		}
		else {
			/*
			 * Start flag not found in index 0 of the message
			 * something is wrong with the message
			 */
		}
	}

	uint8_t escapeByte(uint8_t bite){
		return (bite ^ ESCAPE_MASK);
	}

	//Converts the byte vector into a byte array
	uint8_t* vect2ByteArray(std::vector<uint8_t> byte_vect){
		uint8_t[]* new_byte_array = &byte_vect[0];
		return new_byte_array;
	}
};
