/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef AIRSPEED_H__
#define AIRSPEED_H__

#include <firmament.h>

#ifdef __cplusplus
extern "C" {
#endif

/* mag device bus type */
#define AIRSPEED_SPI_BUS_TYPE 1
#define AIRSPEED_I2C_BUS_TYPE 2

struct airspeed_device {
    struct rt_device parent;
    const struct airspeed_ops* ops;
    rt_uint8_t bus_type;
};
typedef struct airspeed_device* airspeed_dev_t;

/* airspeed driver opeations */
struct airspeed_ops {
    rt_err_t (*dev_control)(airspeed_dev_t dev, int cmd, void* arg);
    rt_size_t (*dev_read)(airspeed_dev_t dev, rt_off_t pos, void* data, rt_size_t size);
};

rt_err_t hal_airspeed_register(airspeed_dev_t dev, const char* name, rt_uint32_t flag, void* data);

#ifdef __cplusplus
}
#endif

#endif
