import os
from glob import glob
from setuptools import setup

package_name = "kogrob"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
        (os.path.join("share", package_name), glob("rviz/*config.rviz")),
        
    ],
    install_requires=["setuptools==58.2.0"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="manuel.weiss@bht-berlin.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "body_publisher_node = kogrob.body_publisher:main",
            "body_tf_subscriber_node = kogrob.body_tf_broadcaster:main",
        ],
    },
)
