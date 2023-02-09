from setuptools import setup, find_packages

setup(
    name='view_animator',
    version='0.1.0',
    packages=find_packages(),
    description="Animate the camera view for a variety of environments",
    url='',
    license='MIT',
    author='zhsh',
    author_email='zhsh@umich.edu',
    test_suite='pytest',
    tests_require=[
        'pytest', 'pybullet',
    ],
    install_requires=[
    ]
)
