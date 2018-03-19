START_FLAG = "x7E"
STOP_FLAG = "x7E"
ESCAPE_FLAG = "x7D"
ESCAPE_MASK = 0x20

#Assuming strBytes is actually a string in the form:
#   "x00x01x02x03"

#Unencapsulates the bytes from the serial wrapping
def serialUnwrap(strBytes):         
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
def serialWrap(strBytes):
    newStrBytes = START_FLAG
    for i in range(0,len(strBytes),3):
        if (strBytes[i:i+3] == ESCAPE_FLAG):
            newStrBytes += ESCAPE_CHAR + escapeStrByte(strBytes[i:i+3])
        else:
            newStrBytes += strBytes[i:i+3]

    newStrBytes += STOP_FLAG
    return newStrBytes 


    
#Escapes the given value
def escapeByte(strByte):
    #To "escape" a value simply do a bitwise xor operation of the value with 0x20
    #The xor operation can undo itself if xor'd again, so it is used to escape, and unescape
    intByte = eval("0" + strByte)
    (intByte ^ ESCAPE_MASK)
