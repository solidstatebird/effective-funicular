
#include "VelocityEncoder.h"

// Yes, all the code is in the header file, to provide the user
// configure options with #define (before they include it), and
// to facilitate some crafty optimizations!

VelocityEncoder_internal_state_t * VelocityEncoder::interruptArgs[];

