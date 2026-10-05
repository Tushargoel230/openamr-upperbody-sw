"""T06: a bad value for any arg fails expansion with a message listing the accepted values."""
import pytest
import xacro

from conftest import ARM_PACKAGES, HARDWARE_PROFILES, MASTS, expand_xml

BAD = [
    ("mast_id", "mast_9999", MASTS),
    ("mast_id", "mast_1500", MASTS),      # a retired nine-position ID must not slip through
    ("mast_id", "", MASTS),
    ("arm_package", "openarm2", ARM_PACKAGES),
    ("arm_package", "OpenArm", ARM_PACKAGES),
    ("hardware_profile", "sim", HARDWARE_PROFILES),
    ("hardware_profile", "", HARDWARE_PROFILES),
    ("enable_planar_base", "yes", ["true", "false"]),
    ("enable_planar_base", "True", ["true", "false"]),
]


@pytest.mark.parametrize("arg,value,accepted", BAD, ids=[f"{a}={v!r}" for a, v, _ in BAD])
def test_t06_bad_value_errors_with_accepted_list(arg, value, accepted):
    with pytest.raises(xacro.XacroException) as err:
        expand_xml(**{arg: value})
    msg = str(err.value)
    assert f"Invalid xacro arg {arg}:={value}" in msg
    assert "Accepted values: " + ", ".join(accepted) in msg


def test_t06_missing_datums_file_errors_naming_the_path():
    with pytest.raises(xacro.XacroException) as err:
        expand_xml(datums_file="/nonexistent/datums.yaml")
    assert "/nonexistent/datums.yaml" in str(err.value)
