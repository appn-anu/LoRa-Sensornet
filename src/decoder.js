
var PAYLOAD_NONE   = 0;
var PAYLOAD_BATT   = (1 << 0);
var PAYLOAD_SOIL   = (1 << 1);
var PAYLOAD_AIR    = (1 << 2);
var PAYLOAD_ANALOG1 = (1 << 3);
var PAYLOAD_ANALOG2 = (1 << 4);
var PAYLOAD_ANALOG3 = (1 << 5);
var PAYLOAD_ANALOG4 = (1 << 6);
var PAYLOAD_ANALOG5 = (1 << 7);

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

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.

    var decoded = {};
    
    var payload_type = bytes[0] + bytes[1] * 256;
    var n = 2

    if (payload_type == PAYLOAD_NONE) {
        // none
    }
    if (payload_type & PAYLOAD_BATT){
        decoded.batt_mV = bytes[n++] + bytes[n++] * 256;    
    }
    
    if (payload_type & PAYLOAD_ANALOG1) {
        // decode analog
        decoded.analog1_mV = bytes[n++] + bytes[n++] * 256;
    }

    if (payload_type & PAYLOAD_ANALOG2) {
        // decode analog
        decoded.analog2_mV = bytes[n++] + bytes[n++] * 256;
    }

    if (payload_type & PAYLOAD_ANALOG3) {
        // decode analog
        decoded.analog3_mV = bytes[n++] + bytes[n++] * 256;
    }

    if (payload_type & PAYLOAD_ANALOG4) {
        // decode analog
        decoded.analog4_mV = bytes[n++] + bytes[n++] * 256;
    }

    if (payload_type & PAYLOAD_ANALOG5) {
        // decode analog
        decoded.analog5_mV = bytes[n++] + bytes[n++] * 256;
    }

    if (payload_type & PAYLOAD_SOIL){
        // decode soil
        decoded.light         = bytes[n++]; // light is single byte
        decoded.soil_tempC    = sflt162f(bytes[n++] + bytes[n++] *256) * 100.0,
        decoded.soil_moisture = bytes[n++] + bytes[n++] * 256
    }

    if (payload_type & PAYLOAD_AIR){
        // decode air
        decoded.air_tempC            = sflt162f(bytes[n++] + bytes[n++] *256) * 100.0;
        decoded.air_relativehumidity = sflt162f(bytes[n++] + bytes[n++] *256) * 1000.0;
        decoded.air_pressurehPa      = sflt162f(bytes[n++] + bytes[n++] *256) * 10000.0;
    }

    return decoded;
}