START_FLAG = 0x7E
STOP_FLAG = 0x7E
ESCAPE_FLAG = 0x7D
ESCAPE_CHAR = 0x20


#Unencapsulates the bytes from the serial wrapping
def serialUnwrap(byteArray):

	index = 1
	newByteArray = []
	
	#Don't start unless we see the start flag in the first element of the byte array
	if (byteArray[index-1] == START_FLAG):
	
		#Continue looping until we see the stop flag
		while (byteArray[index] != STOP_FLAG):
		
			#If we see the escape flag, escape the next byte and add it to the new byte array
			if (byteArray[index] == ESCAPE_FLAG):
				newByteArray.append(escapeByte(byteArray[index+1]))
				index+=2
			#Otherwise add the current byte to the new byte array
			else:
				newByteArray.append(byteArray[index])
				index+=1
				
		#Once the while loop stops, we've finished unwrapping the message
		return newByteArray
	
	

#Encapsulates the bytes with serial wrapping
def serialWrap(byteArray):

	newByteArray = [START_FLAG]
	
	#Iterate through all values in the array to be transmitted
	for byteVal in byteArray:
	
		#If any of the data is the same as a flag, escape the data, and insert the escape flag 
		if ((byteVal == START_FLAG) or (byteVal == STOP_FLAG) or (byteVal == ESCAPE_FLAG)):
			newByteArray.append(ESCAPE_FLAG)
			newByteArray.append(escapeByte(byteVal))
			
		#Otherwise just add the byte to the array
		else:
			newByteArray.append(byteVal)
		
	newByteArray.append(STOP_FLAG)
	return newByteArray
	

	
#Escapes the given value
def escapeByte(byteVal):
	#To "escape" a value simply do a bitwise xor operation of the value with 0x02
	#The xor operation can undo itself if xor'd again, so it is used to escape, and unescape
	return (byteVal ^ ESCAPE_CHAR)
	
			
