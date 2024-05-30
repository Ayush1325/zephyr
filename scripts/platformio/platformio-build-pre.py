# Copyright 2019-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from SCons.Script import AlwaysBuild


Import("env")


def ZephyrBuildProgram(env):
    env["LDSCRIPT_PATH"] = None
    env.ProcessProgramDeps()
    env.ProcessProjectDeps()

    # append into the beginning a main LD script
    env.Prepend(LINKFLAGS=["-T", "$LDSCRIPT_PATH"])

    # enable "cyclic reference" for linker
    if env.get("LIBS") and env.GetCompilerType() == "gcc":
        env.Prepend(_LIBFLAGS="-Wl,--start-group ")
        env.Append(_LIBFLAGS=" -Wl,--end-group")

    program_pre0 = env.Program(
        os.path.join("$BUILD_DIR", "zephyr", "firmware-pre0"),
        env["PIOBUILDFILES"],
        LDSCRIPT_PATH=os.path.join("$BUILD_DIR", "zephyr", "linker_zephyr_pre0.cmd"),
    )

    program_pre1 = env.Program(
        os.path.join("$BUILD_DIR", "zephyr", "firmware-pre1"),
        env["PIOBUILDFILES"] + env["_EXTRA_ZEPHYR_PIOBUILDFILES_PRE1"],
        LDSCRIPT_PATH=os.path.join("$BUILD_DIR", "zephyr", "linker_zephyr_pre1.cmd"),
    )

    # Force execution of offset header target before compiling project sources
    env.Depends(env["PIOBUILDFILES"], env["__ZEPHYR_OFFSET_HEADER_CMD"])

    program = env.Program(
        os.path.join("$BUILD_DIR", env.subst("$PROGNAME")),
        env["PIOBUILDFILES"] + env["_EXTRA_ZEPHYR_PIOBUILDFILES_FINAL"],
        LDSCRIPT_PATH=os.path.join("$BUILD_DIR", "zephyr", "linker.cmd"),
    )

    env.Depends(program_pre1, program_pre0)
    env.Depends(program, program_pre1)

    env.Replace(PIOMAINPROG=program)

    AlwaysBuild(
        env.Alias(
            "checkprogsize",
            program,
            env.VerboseAction(env.CheckUploadSize, "Checking size $PIOMAINPROG"),
        )
    )

    print("Building in %s mode" % env.GetBuildType())

    return program


env.AddMethod(ZephyrBuildProgram, "BuildProgram")
