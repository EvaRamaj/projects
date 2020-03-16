import os

try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup, find_packages

with open('requirements.txt') as f:
    required = f.read().splitlines()

setup(name='wayer',
      version='0.0.0',
      description='Python backend for matching system',
      classifiers=[
        'Programming Language :: Python :: 3.7',
      ],
      keywords='python, flask, nlp, text similarity',
      url='https://github.com/DigitalProductschool/batch5--audios',
      author='wayer',
      author_email='ayamlearning@gmail.com',
      packages=find_packages(),
      install_requires=required)
