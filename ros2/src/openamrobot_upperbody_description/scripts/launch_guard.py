"""What every launch test must check after the launch has stopped: that the processes it needed started, and that none of them died.

`launch_testing` runs the test methods while the launch is up and, unless the module says otherwise, never looks at how the processes ended:
a process of the launch that dies of a signal while the test passes, or one that cannot be started at all, is invisible to the test and so to
the evidence (demonstrated with canaries, P12A task 0: both ended as a passing test and a `0 FAIL` report). So each launch test module ends with

    @launch_testing.post_shutdown_test()
    class TestShutdown(unittest.TestCase):
        def test_processes_started_and_exited_cleanly(self, proc_info):
            assert_launch_healthy(proc_info, expected=["move_group", ...], allowed={"move_group": [SIGSEGV]})

`expected` lists the name prefix of every process the test cannot do without: a process that never started is not in `proc_info` at all, so
only a list kept next to the launch description can notice it is missing. `allowed` lists the non-zero exit codes a process may end with, per
process, each one a known and documented behaviour of the installed software and nothing broader. `test_h01_harness_guards.py` fails any launch
test module that does not call this, so a new launch test cannot be written without it.
"""
from launch_testing.asserts import EXIT_OK, assertExitCodes

SIGSEGV = -11       # move_group, moveit_py: observed on every stop of MoveIt 2.12.4 (see the moveit package README)
SIGINT = -2         # a process the launch stopped with SIGINT and that did not install a handler (a ruby wrapper, `sleep`)
SIGTERM = -15       # the same process, when it did not die of the SIGINT within the launch's timeout and the launch escalated
SIGABRT = -6        # ros_gz_bridge's parameter_bridge, once in a while, at shutdown: see SIM_ALLOWED

# Non-zero exits that are known behaviours of the installed software at shutdown, and nothing broader. Each is a finding, not an approval:
#   gazebo             -2   the `ruby` wrapper of `gz sim` dies of the SIGINT that stops the launch (the server is stopped through /server_control),
#                      -15  or, when it outlives the launch's SIGINT timeout, of the SIGTERM the launch then sends (seen in one G02 run, P12A)
#   parameter_bridge   -6   ros2 ros_gz_bridge aborts on SIGINT with "corrupted double-linked list" in some runs (found in the log of one G01
#                           run in P12A: before this guard nothing looked at how the processes ended). Heap corruption is not benign. It is
#                           allowed here only so that a shutdown abort of platform-sw's bridge does not make G01/G02/M02 flaky, and it is
#                           a finding to take to platform-sw, not an approval
MOVE_GROUP_ALLOWED = {"move_group": [SIGSEGV]}
SIM_ALLOWED = {"gazebo": [SIGINT, SIGTERM], "parameter_bridge": [SIGABRT], **MOVE_GROUP_ALLOWED}


def assert_launch_healthy(proc_info, expected, allowed=None):
    """Fail if a process in `expected` (name prefixes) never started, or if any process that did start exited with a code that is neither 0 nor
    listed for it in `allowed` ({name prefix: [exit codes]})."""
    allowed = allowed or {}
    started = list(proc_info.process_names())
    never_started = [prefix for prefix in expected if not any(name.startswith(prefix) for name in started)]
    assert not never_started, f"processes that never started: {never_started} (the launch started {started})"
    assert started, "the launch started no process at all"
    for name in started:
        codes = [EXIT_OK]
        for prefix, extra in allowed.items():
            if name.startswith(prefix):
                codes += list(extra)
        assertExitCodes(proc_info, allowable_exit_codes=codes, process=name)
