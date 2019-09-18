// decoder.js 
// ttn decoder for this application

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

var payload_types = {
    PAYLOAD_NONE: 0x00,
    PAYLOAD_BATT_ONLY: 0x01,
    PAYLOAD_BATT_SOIL: 0x02,
    PAYLOAD_BATT_AIR: 0x03,
    PAYLOAD_BATT_SOIL_AIR: 0x04,
    PAYLOAD_BATT_ANALOG: 0x05,
    PAYLOAD_BATT_ANALOG_SOIL: 0x06,
    PAYLOAD_BATT_ANALOG_AIR: 0x07,
    PAYLOAD_BATT_ANALOG_SOIL_AIR: 0x08
};

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.

    var decoded = {};

    var payload_type = bytes[0];
    var n = 1
    if (payload_type != payload_types.PAYLOAD_NONE){
        decoded.batt_mV = bytes[n++] + bytes[n++] * 256;    
    }

    if (payload_type >= payload_types.PAYLOAD_BATT_ANALOG){
        decoded.analog_mV = bytes[n++] + bytes[n++] * 256;
    }

    if (payload_type == payload_types.PAYLOAD_BATT_SOIL || 
        payload_type == payload_types.PAYLOAD_BATT_ANALOG_AIR){
        decoded.light         = bytes[n++] + bytes[n++] * 256;
        decoded.soil_tempC    = sflt162f(bytes[n++] + bytes[n++] *256) * 100.0,
        decoded.soil_moisture = bytes[n++] + bytes[n++] * 256
    }

    if (payload_type == payload_types.PAYLOAD_BATT_AIR ||
        payload_type == payload_types.PAYLOAD_BATT_ANALOG_AIR ){
        decoded.air_tempC            = sflt162f(bytes[n++] + bytes[n++] *256) * 100.0;
        decoded.air_relativehumidity = sflt162f(bytes[n++] + bytes[n++] *256) * 1000.0;
        decoded.air_pressureP        = sflt162f(bytes[n++] + bytes[n++] *256) * 10000.0;
    }

    if (payload_type == payload_types.PAYLOAD_BATT_SOIL_AIR ||
        payload_type == payload_types.PAYLOAD_BATT_ANALOG_SOIL_AIR ){
        if (payload_type == payload_types.PAYLOAD_BATT_SOIL_AIR) {
            decoded.light = bytes[n++] + bytes[n++] * 256; // old way to send light
        } else{
            decoded.light = bytes[n++]; // light changes to single byte in new
        }
        decoded.soil_tempC           = sflt162f(bytes[n++] + bytes[n++] *256) * 100.0;
        decoded.soil_moisture        = bytes[n++] + bytes[n++] * 256;
        decoded.air_tempC            = sflt162f(bytes[n++] + bytes[n++] *256) * 100.0;
        decoded.air_relativehumidity = sflt162f(bytes[n++] + bytes[n++] *256) * 1000.0;
        decoded.air_pressurehPa      = sflt162f(bytes[n++] + bytes[n++] *256) * 10000.0;
    }

    return decoded;
}