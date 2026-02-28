# NOTE: this is workaround for misbehaviour of symlink install
#       see the project README.md (the top-level one)
#       for more info (section [Launch files are not symlinked])
from follow_the_gap_v0_ride import launchers


def generate_launch_description():
    return launchers.generate_start_launch_description()
