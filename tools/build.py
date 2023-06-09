#!/usr/bin/env python3

import os
import subprocess

import b
from utility.dockerise import run_on_docker


@run_on_docker
def register(command):
    # Install help
    command.help = "build the codebase"

    command.add_argument("args", nargs="...", help="the arguments to pass through to ninja")
    command.add_argument("-j", help="number of jobs to spawn")


@run_on_docker
def run(j, args, **kwargs):
    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Run cmake if we don't have a ninja build file
    if not os.path.isfile("build.ninja"):
        exitcode = os.system("cmake {} -GNinja".format(b.project_dir)) >> 8

        # If cmake errors return with its status
        if exitcode != 0:
            exit(exitcode)

    # To pass arguments to the ninja command you put them after "--"
    # but "--"  isn't a valid argument for ninja, so we remove it here
    if "--" in args:
        args.remove("--")

    command = ["ninja", *args]

    if j:
        command.insert(1, "-j{}".format(j))

    # Return the exit code of ninja
    exit(subprocess.run(command).returncode)
