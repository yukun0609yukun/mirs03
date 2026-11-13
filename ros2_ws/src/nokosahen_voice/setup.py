from setuptools import find_packages, setup

package_name = "nokosahen_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MIRS",
    maintainer_email="mirs@example.local",
    description="Bridges /nokosahen/announce_text to the host-side voice_bridge (VOICEVOX + Ollama) service.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "voice_node = nokosahen_voice.voice_node:main",
        ],
    },
)
