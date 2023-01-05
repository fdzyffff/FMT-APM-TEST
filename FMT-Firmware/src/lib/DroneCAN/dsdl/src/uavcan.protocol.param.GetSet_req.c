

#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.param.GetSet_req.h>

#include <uavcan.protocol.param.GetSet_res.h>

#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_param_GetSetRequest_encode(struct uavcan_protocol_param_GetSetRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE);
    _uavcan_protocol_param_GetSetRequest_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_protocol_param_GetSetRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_param_GetSetRequest* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_param_GetSetRequest_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_param_GetSetRequest sample_uavcan_protocol_param_GetSetRequest_msg(void) {

    struct uavcan_protocol_param_GetSetRequest msg;






    msg.index = (uint16_t)random_bitlen_unsigned_val(13);






    msg.value = sample_uavcan_protocol_param_Value_msg();






    msg.name.len = (uint8_t)random_range_unsigned_val(0, 92);
    for (size_t i=0; i < msg.name.len; i++) {




        msg.name.data[i] = (uint8_t)random_bitlen_unsigned_val(8);



    }




    return msg;

}
#endif