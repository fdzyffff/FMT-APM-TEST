
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>



#ifdef __cplusplus
extern "C"
{
#endif

#define DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_MAX_SIZE 230
#define DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_SIGNATURE (0x126A47C9C17A8BD7ULL)

#define DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_ID 64



#define DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_ACCEPTED 0

#define DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_TEMPORARILY_REJECTED 1

#define DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_DENIED 2

#define DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_UNSUPPORTED 3

#define DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED 4




struct dronecan_remoteid_SecureCommandResponse {



    uint32_t sequence;



    uint32_t operation;



    uint8_t result;



    struct { uint8_t len; uint8_t data[220]; }data;



};

uint32_t dronecan_remoteid_SecureCommandResponse_encode(struct dronecan_remoteid_SecureCommandResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool dronecan_remoteid_SecureCommandResponse_decode(const CanardRxTransfer* transfer, struct dronecan_remoteid_SecureCommandResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)

static inline void _dronecan_remoteid_SecureCommandResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_remoteid_SecureCommandResponse* msg, bool tao);
static inline void _dronecan_remoteid_SecureCommandResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_remoteid_SecureCommandResponse* msg, bool tao);
void _dronecan_remoteid_SecureCommandResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_remoteid_SecureCommandResponse* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->sequence);

    *bit_ofs += 32;






    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->operation);

    *bit_ofs += 32;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->result);

    *bit_ofs += 8;







    if (!tao) {


        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.len);
        *bit_ofs += 8;


    }

    for (size_t i=0; i < msg->data.len; i++) {




        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.data[i]);

        *bit_ofs += 8;


    }





}

void _dronecan_remoteid_SecureCommandResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_remoteid_SecureCommandResponse* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->sequence);

    *bit_ofs += 32;







    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->operation);

    *bit_ofs += 32;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->result);

    *bit_ofs += 8;








    if (!tao) {


        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.len);
        *bit_ofs += 8;



    } else {

        msg->data.len = ((transfer->payload_len*8)-*bit_ofs)/8;


    }



    for (size_t i=0; i < msg->data.len; i++) {




        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.data[i]);

        *bit_ofs += 8;


    }







}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dronecan_remoteid_SecureCommandResponse sample_dronecan_remoteid_SecureCommandResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
