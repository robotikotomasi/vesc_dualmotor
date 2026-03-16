"""
extra_scripts.py — PlatformIO SCons hook for VESC / ChibiOS build.

ChibiOS .s assembly files (crt0_v7m.s, chcoreasm_v7m.s) use C preprocessor
directives (#if, #define, etc.).  PlatformIO's default assembler command uses
arm-none-eabi-as directly which does NOT run the C preprocessor, so the macros
stay undefined and the assembler fails.

Fix: replace the ASCOM builder to go through arm-none-eabi-gcc with the flag
-x assembler-with-cpp so that the C preprocessor runs first.
"""

Import("env")
import sys

print(">>> extra_scripts.py loaded — patching ASCOM for ChibiOS CPP assembly <<<",
      file=sys.stderr)

# Override the assembly command for .s files so that arm-none-eabi-gcc is used
# instead of arm-none-eabi-as, enabling the C preprocessor (-x assembler-with-cpp).
# ChibiOS crt0_v7m.s / chcoreasm_v7m.s use #if/#define which require CPP.
#   $CC           = arm-none-eabi-gcc
#   $ASFLAGS      = cpu/fpu flags (-mcpu, -mthumb, -mfpu, -mfloat-abi …)
#   $_CPPDEFFLAGS = all -D defines  (from CPPDEFINES)
#   $_CPPINCFLAGS = all -I paths    (from CPPPATH)
env.Replace(
    ASCOM="$CC -x assembler-with-cpp -c $ASFLAGS $_CPPDEFFLAGS $_CPPINCFLAGS $SOURCE -o $TARGET"
)

# PlatformIO ststm32 does NOT propagate -mfpu/-mfloat-abi to LINKFLAGS.
# Without these the linker rejects hard-float object files.
env.Append(LINKFLAGS=["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"])

print(">>> ASCOM set, LINKFLAGS patched with VFP ABI flags <<<", file=sys.stderr)
