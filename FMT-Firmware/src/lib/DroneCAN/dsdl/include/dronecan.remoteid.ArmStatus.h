
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>



#ifdef __cplusplus
extern "C"
{
#endif

#define DRONECAN_REMOTEID_ARMSTATUS_MAX_SIZE 52
#define DRONECAN_REMOTEID_ARMSTATUS_SIGNATURE (0xFEDF72CCF06F3BDDULL)

#define DRONECAN_REMOTEID_ARMSTATUS_ID 20035



#define DRONECAN_REMOTEID_ARMSTATUS_ODID_ARM_STATUS_GOOD_TO_ARM 0

#define DRONECAN_REMOTEID_ARMSTATUS_ODID_ARM_STATUS_FAIL_GENERIC 1




struct dronecan_remoteid_ArmStatus {



    uint8_t status;



    struct { uint8_t len; uint8_t data[50]; }error;



};

uint32_t dronecan_remoteid_ArmStatus_encode(struct dronecan_remoteid_ArmStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool dronecan_remoteid_ArmStatus_decode(const CanardRxTransfer* transfer, struct dronecan_remoteid_ArmStatus* msg);

#if defined(CANARD_DSDLC_INTERNAL)

static inline void _dronecan_remoteid_ArmStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_remoteid_ArmStatus* msg, bool tao);
static inline void _dronecan_remoteid_ArmStatus_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_remoteid_ArmStatus* msg, bool tao);
void _dronecan_remoteid_ArmStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_remoteid_ArmStatus* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->status);

    *bit_ofs += 8;







    if (!tao) {


        canardEncodeScalar(buffer, *bit_ofs, 6, &msg->error.len);
        *bit_ofs += 6;


    }

    for (size_t i=0; i < msg->error.len; i++) {




        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->error.data[i]);

        *bit_ofs += 8;


    }





}

void _dronecan_remoteid_ArmStatus_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_remoteid_ArmStatus* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->status);

    *bit_ofs += 8;








    if (!tao) {


        canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->error.len);
        *bit_ofs += 6;



    } else {

        msg->error.len = ((transfer->payload_len*8)-*bit_ofs)/8;


    }



    for (size_t i=0; i < msg->error.len; i++) {




        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->error.data[i]);

        *bit_ofs += 8;


    }







}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dronecan_remoteid_ArmStatus sample_dronecan_remoteid_ArmStatus_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
