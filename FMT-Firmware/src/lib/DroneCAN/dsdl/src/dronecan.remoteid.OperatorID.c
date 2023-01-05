

#define CANARD_DSDLC_INTERNAL
#include <dronecan.remoteid.OperatorID.h>

#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t dronecan_remoteid_OperatorID_encode(struct dronecan_remoteid_OperatorID* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, DRONECAN_REMOTEID_OPERATORID_MAX_SIZE);
    _dronecan_remoteid_OperatorID_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool dronecan_remoteid_OperatorID_decode(const CanardRxTransfer* transfer, struct dronecan_remoteid_OperatorID* msg) {
    uint32_t bit_ofs = 0;
    _dronecan_remoteid_OperatorID_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct dronecan_remoteid_OperatorID sample_dronecan_remoteid_OperatorID_msg(void) {

    struct dronecan_remoteid_OperatorID msg;






    msg.id_or_mac.len = (uint8_t)random_range_unsigned_val(0, 20);
    for (size_t i=0; i < msg.id_or_mac.len; i++) {




        msg.id_or_mac.data[i] = (uint8_t)random_bitlen_unsigned_val(8);



    }






    msg.operator_id_type = (uint8_t)random_bitlen_unsigned_val(8);







    msg.operator_id.len = (uint8_t)random_range_unsigned_val(0, 20);
    for (size_t i=0; i < msg.operator_id.len; i++) {




        msg.operator_id.data[i] = (uint8_t)random_bitlen_unsigned_val(8);



    }




    return msg;

}
#endif