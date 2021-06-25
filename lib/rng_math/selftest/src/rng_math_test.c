/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <stdio.h>
#include <string.h>

#include "os/mynewt.h"
#include "testutil/testutil.h"
#include "rng_math_test.h"

bool epsilon_same_float(float a, float b)
{
    return (fabs(a - b) < __FLT_EPSILON__);
}

bool epsilon_same_double(double a, double b)
{
    return (fabs(a - b) < __DBL_EPSILON__);
}

TEST_CASE_DECL(path_loss_test)
TEST_CASE_DECL(calc_tof_test)
TEST_CASE_DECL(calc_tof_sym_test)
TEST_CASE_DECL(calc_tof_to_meters_test)

TEST_SUITE(rng_math_test_all)
{
    path_loss_test();
    calc_tof_test();
    calc_tof_to_meters_test();
}

int main(int argc, char **argv)
{
    rng_math_test_all();
    return tu_any_failed;
}
