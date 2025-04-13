#!/bin/bash

STM32_Programmer_CLI -c port=SWD -w firmware-stm.elf -rst
