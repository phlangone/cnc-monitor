/**
 * @file Rtos.h
 * @brief Centraliza a inclusao do Arduino Mbed RTOS na ordem correta.
 *
 * Em projetos PlatformIO baseados em arquivos .cpp, Arduino.h nao e injetado
 * automaticamente como ocorre com sketches .ino. O core deve ser carregado
 * antes do agregador rtos.h para evitar que cabecalhos CMSIS-RTOS de versoes
 * diferentes sejam processados fora da ordem esperada pelo Arduino Mbed.
 */

#pragma once

#include <Arduino.h>
#include <rtos.h>

