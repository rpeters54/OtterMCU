import os
import shutil
import logging

import riscof.utils as utils
from riscof.pluginTemplate import pluginTemplate
import riscof.constants as constants
from riscv_isac.isac import isac

logger = logging.getLogger()

class sail_cSim(pluginTemplate):
    __model__ = "sail_c_simulator"
    __version__ = "0.5.0"

    def __init__(self, *args, **kwargs):
        sclass = super().__init__(*args, **kwargs)

        config = kwargs.get('config')
        if config is None:
            logger.error("Config node for sail_cSim missing.")
            raise SystemExit(1)

        self.num_jobs = str(config['jobs'] if 'jobs' in config else 1)
        self.pluginpath = os.path.abspath(config['pluginpath'])

        self.sail_exe = "sail_riscv_sim"
        self.isa_spec = os.path.abspath(config['ispec']) if 'ispec' in config else ''
        self.platform_spec = os.path.abspath(config['pspec']) if 'ispec' in config else ''
        self.make = config['make'] if 'make' in config else 'make'
        self.sail_config = config['sail_config']

        logger.debug("SAIL CSim plugin initialised using the following configuration.")
        for entry in config:
            logger.debug(entry+' : '+config[entry])
        return sclass

    def initialise(self, suite, work_dir, archtest_env):
        self.suite = suite
        self.work_dir = work_dir

        compile_args = "-march=rv32i_zicsr_zifencei -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles -g"
        self.compile_cmd = (
            f"riscv32-unknown-elf-gcc {compile_args}"
            f" -T {self.pluginpath}/env/link.ld"
            f" -I {self.pluginpath}/env/"
            f" -I {archtest_env}"
             " {0} -o {1}.elf {2}"
        )
        self.objdump_cmd = "riscv32-unknown-elf-objdump -D {0}.elf > {0}.disass"


    def build(self, isa_yaml, platform_yaml):
        ispec = utils.load_yaml(isa_yaml)['hart0']
        self.xlen = ('64' if 64 in ispec['supported_xlen'] else '32')
        self.isa = 'rv' + self.xlen
        self.compile_cmd = self.compile_cmd+' -mabi='+('lp64 ' if 64 in ispec['supported_xlen'] else 'ilp32 ')
        if "I" in ispec["ISA"]:
            self.isa += 'i'
        if "M" in ispec["ISA"]:
            self.isa += 'm'
        if "C" in ispec["ISA"]:
            self.isa += 'c'
        if "F" in ispec["ISA"]:
            self.isa += 'f'
        if "D" in ispec["ISA"]:
            self.isa += 'd'

        # check that all necesary utils exist
        objdump = "riscv32-unknown-elf-objdump"
        if shutil.which(objdump) is None:
            logger.error(objdump+": executable not found. Please check environment setup.")
            raise SystemExit(1)
        compiler = "riscv32-unknown-elf-gcc"
        if shutil.which(compiler) is None:
            logger.error(compiler+": executable not found. Please check environment setup.")
            raise SystemExit(1)
        if shutil.which(self.sail_exe) is None:
            logger.error(self.sail_exe+ ": executable not found. Please check environment setup.")
            raise SystemExit(1)
        if shutil.which(self.make) is None:
            logger.error(self.make+": executable not found. Please check environment setup.")
            raise SystemExit(1)


    def runTests(self, testList, cgf_file=None):

        logging.info("Creating Makefile...")
        if os.path.exists(self.work_dir+ "/Makefile." + self.name[:-1]):
            os.remove(self.work_dir+ "/Makefile." + self.name[:-1])

        make = utils.makeUtil(makefilePath=os.path.join(self.work_dir, "Makefile." + self.name[:-1]))
        make.makeCommand = f"{self.make} -j {self.num_jobs}"

        logging.info(f"Adding Targets:")
        for idx, file in enumerate(testList):
            testentry = testList[file]
            test_path = testentry['test_path']
            test_dir = testentry['work_dir']
            test_name = os.path.basename(test_path)

            echo_cmd = f"echo \"Running Test: {test_name}\""
            cd_cmd = f"cd {test_dir}"

            # substitute all variables in the compile command that we created in the initialize function
            bin_name = 'ref'
            compile_macros = ' -D' + " -D".join(testentry['macros'])
            compile_cmd = self.compile_cmd.format(test_path, bin_name, compile_macros)
            objdump_cmd = self.objdump_cmd.format(bin_name)

            # RISCOF expects the signature to be named as DUT-<dut-name>.signature
            sig_file = os.path.join(test_dir, self.name[:-1] + ".signature")
            sim_cmd = f"{self.sail_exe} --trace-all --config {self.pluginpath}/env/{self.sail_config} --test-signature={sig_file} {bin_name}.elf > {test_name}.log 2>&1"

            cov_str = ' '
            for label in testentry['coverage_labels']:
                cov_str+=' -l '+label

            coverage_cmd = ''
            if cgf_file is not None:
                coverage_cmd = 'riscv_isac --verbose info coverage -d \
                        -t {0}.log --parser-name c_sail -o coverage.rpt  \
                        --sig-label begin_signature  end_signature \
                        --test-label rvtest_code_begin rvtest_code_end \
                        -e ref.elf -c {1} -x{2} {3};'.format(\
                        test_name, ' -c '.join(cgf_file), self.xlen, cov_str)

            # concatenate all commands that need to be executed within a make-target.
            final_cmd = f"@{cd_cmd}; {echo_cmd}; {compile_cmd}; {objdump_cmd}; {sim_cmd}; {coverage_cmd}"
            logging.info(f"  [{idx:3}/{len(testList):3}] - {test_name}")
            make.add_target(final_cmd)

        # execute all targets
        logging.info(f"Beginning Makefile execution with {self.num_jobs} jobs...")
        make.execute_all(self.work_dir)
