SET PATH=D:\keil_v5\ARM\ARMCC\Bin;D:\VMwarePlayer\bin\;E:\ZSP\ZView5.18.0\ide;E:\ZSP\ZView5.18.0\cmdtools\bin;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files\NVIDIA Corporation\NVIDIA NGX;C:\Program Files\NVIDIA Corporation\NVIDIA NvDLISR;D:\java\bin;D:\matlab\runtime\win64;D:\matlab\bin;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files\Microsoft SQL Server\Client SDK\ODBC\110\Tools\Binn\;C:\Program Files (x86)\Microsoft SQL Server\120\Tools\Binn\;C:\Program Files\Microsoft SQL Server\120\Tools\Binn\;C:\Program Files\Microsoft SQL Server\120\DTS\Binn\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Program Files\dotnet\;C:\Program Files\Microsoft SQL Server\130\Tools\Binn\;C:\Program Files\Microsoft SQL Server\Client SDK\ODBC\170\Tools\Binn\;D:\MINGW\bin;E:\airshipper\;E:\git\Git\cmd;C:\Users\86176\AppData\Local\Microsoft\WindowsApps;C:\Program Files\Bandizip\;D:\Microsoft VS Code\bin;D:\Microsoft VS Code Insiders\bin;D:\java\bin;C:\Users\86176\AppData\Local\Programs\Python\Python38-32;C:\Users\86176\AppData\Local\Programs\Python\Python38-32\Scripts;C:\Users\86176\.dotnet\tools;D:\MINGW\bin;D:\PyCharm Community Edition 2021.1.2\bin;;E:\DevEco Studio 3.0.0.800\bin;;E:\Nmap
SET CPU_TYPE=STM32F407VE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=KaijuPilot
SET CPU_CLOCK=0x00B71B00
"D:\keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\main.__i"
"D:\keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_it.__i"
"D:\keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\system_stm32f4xx.__i"
"D:\keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\imu.__i"
"D:\keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\remote_signal.__i"
"D:\keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\att_ctrl.__i"
"D:\keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\flight_ctrl.__i"
"D:\keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\msg_interchange.__i"
"D:\keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "..\OBJ\KaijuPilot_F4.lnp"
"D:\keil_v5\ARM\ARMCC\Bin\fromelf.exe" "..\OBJ\KaijuPilot_F4.axf" --i32combined --output "..\OBJ\KaijuPilot_F4.hex"
