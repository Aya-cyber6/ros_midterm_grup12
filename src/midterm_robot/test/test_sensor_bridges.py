from pathlib import Path

LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "sim.launch.py"
)


def test_sensor_bridges_defined():
    content = LAUNCH_FILE.read_text()

    expected_topics = [
        "/scan",
        "/camera/image_raw",
        "/imu",
        "/bumper_states",
    ]

    for topic in expected_topics:
        assert f"'{topic}@" in content, (
            f"Missing sensor bridge: {topic}"
        )
