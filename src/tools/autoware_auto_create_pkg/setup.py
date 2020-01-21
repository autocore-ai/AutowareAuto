from setuptools import find_packages
from setuptools import setup

package_name = 'autoware_auto_create_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': [
        'template/*',
        'template/design/*',
        'template/include/hello_world/*',
        'template/src/*',
        'template/test/*',
    ]},
    zip_safe=True,
    author='Juan Pablo Samper',
    author_email='jp.samper@apex.ai',
    maintainer='Juan Pablo Samper',
    maintainer_email='jp.samper@apex.ai',
    url='https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='A command line tool to create a boiler-plate package',
    long_description="""\
The package provides a packge template and a script to use the template to\
create a new package based on this template.""",
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autoware_auto_create_pkg = autoware_auto_create_pkg.main:main',
        ],
    },
)
