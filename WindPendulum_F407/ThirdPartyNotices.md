# Third party notices

## 7Semi-INA240

The ADC averaging and explicitly requested zero-offset calibration in
`Core/Src/services/current_sense.c` reference 7Semi's INA240 library, version
1.0.0, revision `08e91db372ada9359ad7998b05d5c53adf5a3fb9`:

https://github.com/7semi-solutions/7Semi-INA240

The project uses its existing C API, 64-bit integer conversion, batch clipping
checks, an explicit zero-current confirmation, and STM32 HAL acquisition.
Arduino source files and dependencies are not built into this firmware.
The upstream MIT notice is reproduced below.

Copyright (c) 2026 7Semi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## PID-Library

The embedded PID implementation in `Core/Src/control/pid.c` is adapted from
Yared Tadesse's PID-Library, revision
`35cbc969f0de5529a36261ee9f497c29f553ecc1`:

https://github.com/yaredtds/PID-Library

The adaptation keeps the discrete PID, filtered derivative, and actuator
anti-windup concepts while using single-precision values and a project-owned
API suitable for the STM32F407 control loop. The upstream project is licensed
under the MIT License. Its copyright and permission notice are reproduced
below.

Copyright (c) 2018 Yared Tadesse

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
