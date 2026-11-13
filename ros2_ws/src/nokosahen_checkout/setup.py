import os
from glob import glob

from setuptools import find_packages, setup

package_name = "nokosahen_checkout"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MIRS",
    maintainer_email="mirs@example.local",
    description="Per-menu checkout camera unit (Webcam + Raspberry Pi 4) node for Nokosahen.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "checkout_camera = nokosahen_checkout.checkout_camera_node:main",
            "mock_checkout_cam = nokosahen_checkout.mock_checkout_cam_node:main",
        ],
    },
)
