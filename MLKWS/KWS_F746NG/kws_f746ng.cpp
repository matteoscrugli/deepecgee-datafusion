/*
 * Copyright (C) 2018 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kws_f746ng.h"

KWS_F746NG::KWS_F746NG(int recording_win, int sliding_window_len)
:KWS_DS_CNN(recording_win, sliding_window_len)
{
  audio_buffer = new int16_t[audio_buffer_size];
}

KWS_F746NG::~KWS_F746NG()
{
  delete audio_buffer;
}
