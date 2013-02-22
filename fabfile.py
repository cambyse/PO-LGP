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
    local("sed -e '21s/^.*$/MAKEMODE = mrllib_ubuntu/g' share/make-config.default > share/make-config")


def set_build_minimal():
    """Set MAKEMODE in make-generic to 'mlrlib_minimal' """
    local("sed -e '21s/^.*$/MAKEMODE = mrllib_minimal/g' share/make-config.default > share/make-config")


def set_build_full():
    """Set MAKEMODE in make-generic to 'mlrlib_full' """
    local("sed -e '21s/^.*$/MAKEMODE = mrllib_full/g' share/make-config.default > share/make-config")


def rm_lib():
    """rm all files in share/lib/"""
    local("rm -rf share/lib/*")


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
        local("make")


def make_test():
    """Make share/test/"""
    with lcd("share/test/"):
        local("make")


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
