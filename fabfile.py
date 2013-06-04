"""
Helper to execute several jobs (like "make", "clean", "test execution")
in the MLR repository.

This script is mostly used by Jenkins.

Usage:

    fab -l         # list all commands
    fab -h         # display help
    fab cmd_name   # execute command cmd_name
    fab cmd1 cmd2  # execute multiple commands
"""

from fabric.api import local
from fabric.api import lcd


def set_build_ubuntu():
    """Set MAKEMODE in make-generic to 'mlrlib_ubuntu' """
    local("sed '0,/RE/s/^MAKEMODE = mlrlib_ubuntu$/MAKEMODE = mlrlib_ubuntu/' share/make-config.default > share/make-config")


def set_build_minimal():
    """Set MAKEMODE in make-generic to 'mlrlib_minimal' """
    local("sed '0,/RE/s/^MAKEMODE = mlrlib_ubuntu$/MAKEMODE = mlrlib_minimal/' share/make-config.default > share/make-config")


def set_build_full():
    """Set MAKEMODE in make-generic to 'mlrlib_full' """
    local("sed '0,/RE/s/^MAKEMODE = mlrlib_ubuntu$/MAKEMODE = mlrlib_full/' share/make-config.default > share/make-config")


def enable_physx():
    """Enable the use of PhysX in make-config"""
    local("sed -i 's/PHYSX = 0/PHYSX = 1/g' share/make-config")


def rm_lib():
    """rm all files in share/lib/"""
    local("rm -rf share/lib/*")
    local("mkdir -p share/lib")


def clean_src():
    """rm all files in share/src"""
    with lcd("share"):
        local("make cleanAll")


def clean_test():
    """rm all files in share/test"""
    with lcd("share/test/"):
        local("make clean")


def make_src():
    """Make share/src/"""
    with lcd("share"):
        # somehow sh does not like this. but why is it running in sh anyway?
        # local("make > >(tee ../gcc_stdout.log) 2> >(tee ../gcc_warnings.log >&2)")
        # TODO dirty trick: run it twice :(
        local("make")
        local("make > ../gcc_stdout.log")
    make_orspy()


def make_test():
    """Make share/test/"""
    with lcd("share/test/"):
        local("make")


def make_orspy():
    """Make the python bindings of ors: orspy"""
    with lcd("share/src/MT/"):
        local("make -f Makefile_ors_for_swig")


def run_unittests():
    """Run the unittests"""
    with lcd("share/unittest/orspy/"):
        local("nosetests --with-xunit")
    # TODO add gtest as well


def jenkins_clean_build_main():
    """Jenkins builds share/ from sratch"""
    rm_lib()
    clean_src()
    make_src()


def jenkins_clean_build_test():
    """Jenkins builds everything and creates a JUnit file for Jenkins"""
    jenkins_clean_build_main()
    clean_test()
    with lcd("share/test/"):
        local("python jenkins_run_tests.py")


def jenkins_stats():
    """Collect some stats for Jenkins."""
    # cccc
    local("cccc share/src/*/*.cpp share/src/*/*.h share/src/*/*.cxx")

    # doxygen
    with lcd('share/doc/'):
        local('doxygen MLR.doxy')
