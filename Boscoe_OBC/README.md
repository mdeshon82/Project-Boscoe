# Boscoe_OBC — Build, Test, and Flash

This README documents the toolchain and a set of reliable commands to configure, build, test, and upload the firmware for the STM32F411-based `Boscoe_OBC` project.

The repository already contains a `cmake/gcc-arm-none-eabi.cmake` toolchain file. Two common approaches to get a reproducible clean build are shown below.

## Recommended toolchain (works on this machine)
- STM32Cube `gnu-tools-for-stm32` bundle (bundled with STM32CubeIDE / STM32CubeProgrammer). Example path on macOS (adjust for your install):
  - `/Users/me/Library/Application Support/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin`

This bundle was used to produce a successful clean build in this workspace. To use it, prepend the `bin` directory to `PATH` before running CMake, or set `CMAKE_C_COMPILER`/`CMAKE_CXX_COMPILER` explicitly.

## Alternative toolchain (system-installed via Homebrew)
- Homebrew `arm-none-eabi-gcc` can work but may require installing `newlib` / ensuring the compiler's sysroot and C headers are present. If you see errors like `fatal error: stdint.h: No such file or directory`, your cross toolchain's sysroot or newlib headers are missing or not being found.

If you prefer the Homebrew toolchain, ensure it provides a complete sysroot or update `cmake/gcc-arm-none-eabi.cmake` to set `CMAKE_SYSROOT` / `CMAKE_FIND_ROOT_PATH` to the toolchain's sysroot.

---

## Clean configure + build (recommended)
Use Ninja generator for fast builds. These commands will create a clean `build/Debug` directory and produce `.elf`, `.hex`, and `.bin` artifacts.

Bash example (replace the bundled toolchain path if different):

```bash
# 1) Remove any previous build directory
rm -rf build/Debug

# 2) Prepend the STM32Cube bundled toolchain to PATH (adjust path as needed)
export PATH="/Users/me/Library/Application Support/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin:$PATH"

# 3) Configure with CMake (uses the provided toolchain file)
cmake -G Ninja -S . -B build/Debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

# 4) Build
ninja -C build/Debug -v
```

Expected artifacts (after a successful build):
- `build/Debug/Boscoe_OBC.elf`
- `build/Debug/Boscoe_OBC.hex`
- `build/Debug/Boscoe_OBC.bin`
- `build/Debug/Boscoe_OBC.map`

Verify artifacts:

```bash
ls -l build/Debug/Boscoe_OBC.*
arm-none-eabi-size build/Debug/Boscoe_OBC.elf
```

If you want to force explicit compiler paths instead of relying on `PATH`, pass these variables to CMake (example with the bundled toolchain):

```bash
cmake -G Ninja -S . -B build/Debug -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER="/Users/me/Library/Application Support/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin/arm-none-eabi-gcc" \
  -DCMAKE_CXX_COMPILER="/Users/me/Library/Application Support/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin/arm-none-eabi-g++" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
```

---

## Fast build (incremental)
If you already have a configured build dir and want a quick incremental build:

```bash
ninja -C build/Debug
```

---

## Test / Basic checks
- Run the binary size summary (already used in the linker step):

```bash
arm-none-eabi-size build/Debug/Boscoe_OBC.elf
```

- Print the symbol map (linker produces `Boscoe_OBC.map` in build dir):

```bash
less build/Debug/Boscoe_OBC.map
```

- You can inspect the hex or binary if needed:

```bash
hexdump -C build/Debug/Boscoe_OBC.bin | head
```

---

## Common flashing/upload commands
Choose the tool that matches the programmer/debugger you use (ST-Link, STM32CubeProgrammer, OpenOCD, etc.). The examples below assume the target base Flash address is `0x08000000` (STM32 default) and that the build artifacts are in `build/Debug`.

1) stlink / `st-flash` (simple USB ST-Link flasher):

```bash
# write binary at address 0x08000000
st-flash write build/Debug/Boscoe_OBC.bin 0x08000000

# alternatively write ELF (st-flash may accept ELF on some versions)
st-flash write build/Debug/Boscoe_OBC.elf 0x08000000
```

2) STM32CubeProgrammer (GUI or CLI `STM32_Programmer_CLI`) — recommended for official flashing:

```bash
# Use the CLI to connect over ST-Link (SWD) and write the ELF
STM32_Programmer_CLI -c port=SWD -w build/Debug/Boscoe_OBC.elf

# Or write binary at the address
STM32_Programmer_CLI -c port=SWD -w build/Debug/Boscoe_OBC.bin 0x08000000
```

3) OpenOCD + gdb (for debugging or flashing via OpenOCD):

```bash
# start openocd in one terminal (adjust interface/target cfg)
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg

# in another terminal, connect with arm-none-eabi-gdb
arm-none-eabi-gdb build/Debug/Boscoe_OBC.elf
# then in gdb:
# (gdb) target remote localhost:3333
# (gdb) monitor reset halt
# (gdb) load
# (gdb) monitor reset init
# (gdb) continue
```

Notes:
- `openocd` config files vary depending on your ST-Link version and OpenOCD installation.
- `STM32_Programmer_CLI` comes with ST's STM32CubeProgrammer installer.

---

## Troubleshooting notes
- If the build fails with **multiple definition** linker errors, it's typically because the same generated/STM32 HAL/system sources were added twice. This repository was updated to remove duplicated source listings from the top-level `CMakeLists.txt` so `cmake/stm32cubemx` supplies the generated sources once.

- If the build fails with errors like `fatal error: stdint.h: No such file or directory` when using a system/homebrew `arm-none-eabi-gcc`, your cross toolchain may be missing newlib headers or a proper sysroot. Fix options:
  - Use the STM32Cube bundled toolchain (prepend its `bin/` to `PATH`).
  - Or update `cmake/gcc-arm-none-eabi.cmake` to set `CMAKE_SYSROOT`/`CMAKE_FIND_ROOT_PATH` for the Homebrew toolchain.

### Example snippet to set sysroot in `cmake/gcc-arm-none-eabi.cmake` (only if you know the path):
```cmake
# inside cmake/gcc-arm-none-eabi.cmake
set(CMAKE_SYSROOT "/opt/homebrew/Cellar/arm-none-eabi-gcc/<version>/arm-none-eabi")
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
```
Adjust the path for your installed toolchain. (If you want, I can inspect and suggest the exact values for the Homebrew package on your machine.)

---

## If you want me to continue
- I can create a git commit for this `README.md` and the earlier `CMakeLists.txt` change; tell me the branch/commit message you prefer.
- I can also add a short `TOOLCHAIN.md` file showing how to install a reproducible toolchain on a fresh macOS machine.

---

Project: `Boscoe_OBC` — STM32F411xE (Cortex-M4, FPv4)
