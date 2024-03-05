from setuptools import setup, find_packages



setup(
    name="fast_fly",
    version="0.1",
    author="ZXW2600",
    author_email="zhaoxinwei74@gmail.com",
    description="A Fast Trajectory Planning and Tracking Tool",


    # 你要安装的包，通过 setuptools.find_packages 找到当前目录下有哪些包
    packages=find_packages(),
    install_requires=[
        'casadi',
        'matplotlib',
        'pyyaml',
        'coloredlogs'
    
    ],)
