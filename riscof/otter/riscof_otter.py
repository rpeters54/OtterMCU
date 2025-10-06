import os
import json
import subprocess
import logging

import riscof.utils as utils
import riscof.constants as constants
from riscof.pluginTemplate import pluginTemplate

logger = logging.getLogger()

class otter(pluginTemplate):
    __model__ = "otter"
    __version__ = "0.0.1"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        logging.info("Parsing config.ini arguments...")

        config = kwargs.get('config')
        if config is None:
            print("Please enter input file paths in configuration.")
            raise SystemExit(1)

        # Parallel jobs forked by the makefile
        self.num_jobs = str(config['jobs'] if 'jobs' in config else 1)

        # Path to the directory where this python file is located. Collect it from the config.ini
        self.pluginpath=os.path.abspath(config['pluginpath'])
        self.tb_include_paths = [os.path.abspath(path) for path in json.loads(config['tb_include_paths'])]
        self.tb_name = config['tb_name']

        # Collect the paths to the spec YAML files from config.ini
        self.isa_spec = os.path.abspath(config['ispec'])
        self.platform_spec = os.path.abspath(config['pspec'])

        # Check if running tests should be skipped
        if 'target_run' in config and config['target_run']=='0':
            self.target_run = False
        else:
            self.target_run = True

        logging.info("Done.")

    def initialise(self, suite, work_dir, archtest_env):

        logging.info("Formatting compilation/object-builder commands and building DUT...")

        # Specify the dut test working directory
        self.work_dir = work_dir
        # capture the architectural test-suite directory.
        self.suite_dir = suite

        # Create but do not run the firmware compilation and hex-ify commands
        compile_args = "-march=rv32i_zicsr_zifencei -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles -g"
        self.compile_cmd = (
            f"riscv32-unknown-elf-gcc {compile_args}"
            f" -T {self.pluginpath}/env/link.ld"
            f" -I {self.pluginpath}/env/"
            f" -I {archtest_env}"
             " {0} -o {1}.elf {2}"
        )
        objcopy_args = "-O verilog --verilog-data-width=4"
        self.objcopy_cmd = (
            f"riscv32-unknown-elf-objcopy {objcopy_args}"
             " {0}.elf {0}.hex"
        )

        # Build the verilator executable
        self.dut_exe = f"{self.pluginpath}/env/obj_dir/tb_otter"
        verilator_args = "--binary --timing --trace --trace-structs --assert --timescale 1ns --quiet"
        verilator_includes = " ".join([f"-I{path}" for path in self.tb_include_paths])
        dut_build_cmd = (
            f"cd {self.pluginpath}/env/;"
            f" verilator {verilator_args} {verilator_includes}"
            f" -o {self.tb_name} {self.tb_name}.sv >> /dev/null"
        )
        subprocess.run(dut_build_cmd, shell=True, check=True)

        logging.info("Done.")


    def build(self, isa_yaml, platform_yaml):

        logging.info("Retrieving build information from YAML...")
        # load the isa yaml as a dictionary in python.
        ispec = utils.load_yaml(isa_yaml)['hart0']

        # capture the XLEN value by picking the max value in 'supported_xlen' field of isa yaml. This
        # will be useful in setting integer value in the compiler string (if not already hardcoded);
        self.xlen = ('64' if 64 in ispec['supported_xlen'] else '32')

        # for otter start building the '--isa' argument. the self.isa is dutnmae specific and may not be
        # useful for all DUTs
        self.isa = 'rv' + self.xlen
        if "I" in ispec["ISA"]:
            self.isa += 'i'
        if "M" in ispec["ISA"]:
            self.isa += 'm'
        if "F" in ispec["ISA"]:
            self.isa += 'f'
        if "D" in ispec["ISA"]:
            self.isa += 'd'
        if "C" in ispec["ISA"]:
            self.isa += 'c'

        self.compile_cmd = self.compile_cmd+' -mabi='+('lp64 ' if 64 in ispec['supported_xlen'] else 'ilp32 ')
        logging.info("Done.")


    def runTests(self, testList):

        logging.info("Creating Makefile...")
        # Delete Makefile if it already exists.
        if os.path.exists(self.work_dir+ "/Makefile." + self.name[:-1]):
            os.remove(self.work_dir+ "/Makefile." + self.name[:-1])

        # create an instance the makeUtil class that we will use to create targets.
        make = utils.makeUtil(makefilePath=os.path.join(self.work_dir, "Makefile." + self.name[:-1]))

        # set the make command that will be used. The num_jobs parameter was set in the __init__
        # function earlier
        make.makeCommand = f'make -k -j{self.num_jobs}'

        for testname in testList:
            testentry = testList[testname]
            # path to the assembly file of this test
            test_path = testentry['test_path']
            # location to dump test artifacts
            test_dir = testentry['work_dir']

            echo_cmd = f"echo \"Running Test: {os.path.basename(test_path)}\""
            cd_cmd = f"cd {test_dir}"

            # substitute all variables in the compile command that we created in the initialize function
            bin_name = "testbin"
            hex_name = f"{bin_name}.hex"
            # add all gcc compiler macros for the given test
            compile_macros= ' -D' + " -D".join(testentry['macros'])
            compile_cmd = self.compile_cmd.format(test_path, bin_name, compile_macros)
            objcopy_cmd = self.objcopy_cmd.format(bin_name)

            # RISCOF expects the signature to be named as DUT-<dut-name>.signature
            sig_file = os.path.join(test_dir, self.name[:-1] + ".signature")
            if self.target_run:
                sim_cmd = f'{self.dut_exe} +MEMFILE={hex_name} +SIGFILE={sig_file} > run.log'
            else:
                sim_cmd = 'echo "NO RUN"'

            # concatenate all commands that need to be executed within a make-target.
            final_cmd = f"@{cd_cmd}; {echo_cmd}; {compile_cmd}; {objcopy_cmd}; {sim_cmd};"
            logging.info(f"Added Target {test_path}; outputs to {test_dir}")
            make.add_target(final_cmd)

        # execute all targets
        logging.info(f"Beginning Makefile execution with {self.num_jobs} jobs...")
        make.execute_all(self.work_dir)

        if not self.target_run:
            raise SystemExit(0)

        logging.info("Done.")
