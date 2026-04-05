# Tests disabled by default to be skipped on ROS build farm.
# To run locally: pytest test/ --override-ini="collect_ignore_glob="
collect_ignore_glob = ["test/*"]
