from distutils.core import setup

setup(
    # Application name:
    name="BoidModel",

    # Version number:
    version="0.1.0",

    # Application Author Details

    # Packages
    packages=["app"],

    # Include additional files into the package
    include_package_data=True,

    # What the project relates to
    keywords='computational modeling',

    install_requires=['pygame','numpy']
    
)
