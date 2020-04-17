/* Copyright 2020, Edwin Chiu

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef SERIALIO_H
#define SERIALIO_H

#include <stdint.h>

#include "checksum.h"
#include "comms.h"
#include "packet_types.h"

void serialIO_init();
void serialIO_send(enum msgType type, enum dataID id, char *data, uint8_t len);
bool serialIO_dataAvailable();
void serialIO_readByte(char *buffer);

#endif // SERIALIO_H
