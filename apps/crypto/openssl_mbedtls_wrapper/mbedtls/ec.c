/****************************************************************************
 * apps/crypto/openssl_mbedtls_wrapper/mbedtls/ec.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <openssl/ec.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void EC_KEY_free(EC_KEY *a)
{
}

EC_GROUP *EC_GROUP_new_by_curve_name(int nid)
{
  return NULL;
}

void EC_GROUP_free(EC_GROUP *a)
{
}

int EC_GROUP_get_curve_name(const EC_GROUP *group)
{
  return 0;
}
