# SPDX-FileCopyrightText: 2022 ShinagwaKazemaru
# SPDX-FileCopyrightIdentifer: MIT License

import os
from glob import glob
from setuptools import setup

package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shinagawa kazemaru',
    maintainer_email='s18c1054uc@s.chibakoudai.jp',
    description='package for practice',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'talker = mypkg.talker:main',
			'listener = mypkg.listener:main',
			'result = mypkg.result:main',
			'prime_factorizer = mypkg.prime_factorizer:main',
			'sqsum_calculator = mypkg.sqsum_calculator:main',
        ],
    },
)
