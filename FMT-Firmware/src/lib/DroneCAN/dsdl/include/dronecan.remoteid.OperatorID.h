
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>



#ifdef __cplusplus
extern "C"
{
#endif

#define DRONECAN_REMOTEID_OPERATORID_MAX_SIZE 43
#define DRONECAN_REMOTEID_OPERATORID_SIGNATURE (0x581E7FC7F03AF935ULL)

#define DRONECAN_REMOTEID_OPERATORID_ID 20034



#define DRONECAN_REMOTEID_OPERATORID_ODID_OPERATOR_ID_TYPE_CAA 0




struct dronecan_remoteid_OperatorID {



    struct { uint8_t len; uint8_t data[20]; }id_or_mac;



    uint8_t operator_id_type;



    struct { uint8_t len; uint8_t data[20]; }operator_id;



};

uint32_t dronecan_remoteid_OperatorID_encode(struct dronecan_remoteid_OperatorID* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool dronecan_remoteid_OperatorID_decode(const CanardRxTransfer* transfer, struct dronecan_remoteid_OperatorID* msg);

#if defined(CANARD_DSDLC_INTERNAL)

static inline void _dronecan_remoteid_OperatorID_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_remoteid_OperatorID* msg, bool tao);
static inline void _dronecan_remoteid_OperatorID_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_remoteid_OperatorID* msg, bool tao);
void _dronecan_remoteid_OperatorID_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_remoteid_OperatorID* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;







    canardEncodeScalar(buffer, *bit_ofs, 5, &msg->id_or_mac.len);
    *bit_ofs += 5;

    for (size_t i=0; i < msg->id_or_mac.len; i++) {




        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->id_or_mac.data[i]);

        *bit_ofs += 8;


    }






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->operator_id_type);

    *bit_ofs += 8;







    if (!tao) {


        canardEncodeScalar(buffer, *bit_ofs, 5, &msg->operator_id.len);
        *bit_ofs += 5;


    }

    for (size_t i=0; i < msg->operator_id.len; i++) {




        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->operator_id.data[i]);

        *bit_ofs += 8;


    }





}

void _dronecan_remoteid_OperatorID_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_remoteid_OperatorID* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;







    canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->id_or_mac.len);
    *bit_ofs += 5;


    for (size_t i=0; i < msg->id_or_mac.len; i++) {




        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->id_or_mac.data[i]);

        *bit_ofs += 8;


    }








    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->operator_id_type);

    *bit_ofs += 8;








    if (!tao) {


        canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->operator_id.len);
        *bit_ofs += 5;



    } else {

        msg->operator_id.len = ((transfer->payload_len*8)-*bit_ofs)/8;


    }



    for (size_t i=0; i < msg->operator_id.len; i++) {




        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->operator_id.data[i]);

        *bit_ofs += 8;


    }







}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dronecan_remoteid_OperatorID sample_dronecan_remoteid_OperatorID_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
