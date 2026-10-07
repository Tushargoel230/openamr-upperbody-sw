"""H01: the two ways a test can exist and still tell the evidence nothing, closed statically (P12A task 0).

1. A test file that is not registered in its package's CMakeLists.txt never runs, and nothing reports a test that never ran. Every
   `test_*.py` of the five packages that contribute checks must be registered.
2. A launch test whose module does not look at how its processes ended passes while a process of the launch crashes or never starts
   (demonstrated with canaries). Every launch test module must end with a post-shutdown test that calls `assert_launch_healthy`
   (openamrobot_upperbody_description/scripts/launch_guard.py) and name the processes it cannot do without.

The check is on the source tree: it reads the packages' CMakeLists.txt and test directories next to this one, so a canary planted in any of them
is seen, and a new launch test cannot be added without the guard."""
import os
import re

import pytest

SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PACKAGES = [f"openamrobot_upperbody_{name}" for name in ("description", "control", "bringup", "gazebo", "moveit")]
REGISTERED = re.compile(r"(?:ament_add_pytest_test\(\s*\S+\s+|add_launch_test\(\s*)(?P<path>test/\S+\.py)")


def read(path):
    with open(path) as f:
        return f.read()


def registered_files(package):
    return {os.path.basename(m.group("path")) for m in REGISTERED.finditer(read(os.path.join(SRC, package, "CMakeLists.txt")))}


def list_test_files(package):
    directory = os.path.join(SRC, package, "test")
    return sorted(n for n in os.listdir(directory) if n.startswith("test_") and n.endswith(".py")) if os.path.isdir(directory) else []


def list_launch_tests():
    return [(p, n) for p in PACKAGES for n in list_test_files(p)
            if re.search(r"^def generate_test_description\(", read(os.path.join(SRC, p, "test", n)), flags=re.M)]


@pytest.mark.parametrize("package", PACKAGES)
def test_h01_every_test_file_is_registered_in_its_packages_cmakelists(package):
    unregistered = sorted(set(list_test_files(package)) - registered_files(package))
    assert not unregistered, f"{package}: test files that no CMake line registers, so they never run: {unregistered}"


def test_h01_there_are_launch_tests_to_guard():
    names = [n for _, n in list_launch_tests()]
    for expected in ("test_l01_mock_controllers.py", "test_m01_mock_named_poses.py", "test_m02_gazebo_named_poses.py", "test_g01_gazebo_smoke.py",
                     "test_g02_gripper_mimic.py", "test_m04_collision_goal_rejected.py", "test_joint_state_clamp_loop.py"):
        assert expected in names, f"{expected} is not recognised as a launch test: the scan below would be vacuous"


@pytest.mark.parametrize("package,name", list_launch_tests() or [("none", "none")])
def test_h01_every_launch_test_checks_that_its_processes_started_and_exited_cleanly(package, name):
    text = read(os.path.join(SRC, package, "test", name))
    assert "launch_testing.post_shutdown_test" in text, f"{package}/test/{name}: no post-shutdown test: a crashed or never-started process would pass"
    assert re.search(r"assert_launch_healthy\(|stack_healthy\(", text), \
        f"{package}/test/{name}: no call to assert_launch_healthy (launch_guard.py) after the launch stops"
    assert re.search(r"assert_no_process_died\(|stack_alive\(", text), \
        f"{package}/test/{name}: no active test that no process died WHILE the tests ran (assert_no_process_died): the post-shutdown allowances would hide it"
    assert re.search(r"expected\s*=\s*\[|EXPECTED_PROCESSES", text), f"{package}/test/{name}: it does not name the processes it needs (expected=[...])"
