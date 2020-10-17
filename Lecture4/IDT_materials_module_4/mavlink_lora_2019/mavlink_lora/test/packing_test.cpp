#include <gtest/gtest.h>
#include <vector>
extern "C"
{
    #include "serial.h"
    #include "mavlink_lora_lib.h"
}

/*
Note the use of EXPECT_* macros, instead of ASSERT_* macros. Whereas the ASSERT_* macros records failure and immediately exits from the test, the EXPECT_* version records failure and continues. The latter behavior is usually what you want, because you want to run every test. The exception would be in the case where one test depends on the successful execution of a previous test.
*/

// add extern dummy functions the library expects to be able to compile
void ml_parse_msg(unsigned char *msg) {}
extern void ml_tx_update (void) {}

// write gtests
TEST(Unpack, heartbeat){

    //make object, pack it and then unpack and check values are same
    ml_queue_msg_heartbeat(8, 5, 14, 512, 7, 0);

    //the msg is queued and packed into txbuf. send it to unpack from payload mark
    std::vector<uint8_t> payload;
    uint8_t payload_len = txbuf[ML_POS_PAYLOAD_LEN];

    //copy payload over
    for (auto i=0; i < payload_len; i++)
        payload.push_back(txbuf[ML_POS_PAYLOAD + i]);

    //unpack payload
    mavlink_heartbeat_t h = ml_unpack_msg_heartbeat(&payload.front());

    //expect length to be equal
    EXPECT_EQ(payload_len, MAVLINK_MSG_ID_HEARTBEAT_LEN);

    //expect values to be same as what we queued
    EXPECT_EQ(8, h.type);
    EXPECT_EQ(5, h.autopilot);
    EXPECT_EQ(14, h.base_mode);
    EXPECT_EQ(512, h.custom_mode);
    EXPECT_EQ(7, h.system_status);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
