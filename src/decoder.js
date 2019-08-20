// decoder.js 
// ttn decoder for this application
/*
void build_data() {
  size_t n = 0;
  dataTX[n++] = lowByte(batt_mV);
  dataTX[n++] = highByte(batt_mV);
  dataTX[n++] = lowByte(light);
  dataTX[n++] = highByte(light);
  uint16_t payloadTemp = LMIC_f2sflt16(tempC/100.0); // adjust for the f2sflt16 range (-1 to 1)
  dataTX[n++] = lowByte(payloadTemp);
  dataTX[n++] = highByte(payloadTemp);
  dataTX[n++] = lowByte(moisture);
  dataTX[n++] = highByte(moisture);
}
*/
// light:15 temp:23.60 moisture:240 battery:4473

// 79110F008D6FF0

function sflt162f(rawSflt16){
    // rawSflt16 is the 2-byte number decoded from wherever;
    // it's in range 0..0xFFFF
    // bit 15 is the sign bit
    // bits 14..11 are the exponent
    // bits 10..0 are the the mantissa. Unlike IEEE format,
    // the msb is explicit; this means that numbers
    // might not be normalized, but makes coding for
    // underflow easier.
    // As with IEEE format, negative zero is possible, so
    // we special-case that in hopes that JavaScript will
    // also cooperate.
    //
    // The result is a number in the open interval (-1.0, 1.0);
    //

    // throw away high bits for repeatability.
    rawSflt16 &= 0xFFFF;

    // special case minus zero:
    if (rawSflt16 == 0x8000)
        return -0.0;

    // extract the sign.
    var sSign = ((rawSflt16 & 0x8000) != 0) ? -1 : 1;

    // extract the exponent
    var exp1 = (rawSflt16 >> 11) & 0xF;

    // extract the "mantissa" (the fractional part)
    var mant1 = (rawSflt16 & 0x7FF) / 2048.0;

    // convert back to a floating point number. We hope
    // that Math.pow(2, k) is handled efficiently by
    // the JS interpreter! If this is time critical code,
    // you can replace by a suitable shift and divide.
    var f_unscaled = sSign * mant1 * Math.pow(2, exp1 - 15);

    return f_unscaled;
    }

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.

    
    var decoded = {
        batt_mV     : bytes[0] + bytes[1] * 256,
        light       : bytes[2] + bytes[3] * 256,
        tempC       : sflt162f(bytes[4] + bytes[5] *256) * 100.0,
        moisture    : bytes[6] + bytes[7] * 256
    };
    

    return decoded;
  }