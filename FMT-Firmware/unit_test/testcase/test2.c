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
#include <firmament.h>

#include <utest.h>

static void test_unit(void)
{
    int a = 1;
    int b = 1;
    char* str1 = "test";
    char* str2 = "test";

    uassert_int_equal(a, b);
    uassert_str_equal(str1, str2);
}

static rt_err_t testcase_init(void)
{
    return RT_EOK;
}

static rt_err_t testcase_cleanup(void)
{
    return RT_EOK;
}

static void testcase(void)
{
    UTEST_UNIT_RUN(test_unit);
}
UTEST_TC_EXPORT(testcase, "unit_test.sample.test2", testcase_init, testcase_cleanup, 1000);